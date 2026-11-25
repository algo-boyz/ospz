package main

// Little SPZ / PLY splat viewer.
//
//   odin/
//     ospz/
//     ospz-viewer/    <- this file
//
//   cd ospz-viewer
//   odin run . -- ../ospz/models/boat.spz
//
// Opens .spz and 3DGS .ply (binary little-endian, float properties) files; the
// format is detected from the file contents.
// Drag a .spz or .ply onto the window to swap models.
// Left-drag orbits, wheel zooms, R frames the model again.
//
// Splats are camera-projected 3D Gaussians (scale + quaternion), blended
// back-to-front. Higher-order spherical harmonics are not shaded; colour is
// the DC term, same as a degree-0 preview.

import "core:fmt"
import "core:math"
import "core:os"
import "core:slice"

import ospz ".."
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"

DATA_W :: 1024
ORDER_W :: 1024
SH_C0 :: f32(0.28209479177387814)

Splat_View :: struct {
	count:     int,
	center:    [3]f32,
	extent:    f32,
	name:      string,
	data_tex:  rl.Texture2D,
	order_tex: rl.Texture2D,
	order_px:  []u8,
	pos:       [][3]f32,
	order:     []int,
	xforms:    []rl.Matrix,
	data_w:    i32,
	data_h:    i32,
	order_h:   i32,
}

g_cam: [3]f32
g_pos: [][3]f32

main :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .MSAA_4X_HINT, .VSYNC_HINT})
	rl.InitWindow(1280, 720, "ospz")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	path := "tree.ply"
	if len(os.args) > 1 {
		path = os.args[1]
	}

	view, load_err := load_view(path)
	defer release_view(&view)

	shader := rl.LoadShaderFromMemory(VS, FS)
	if shader.id == 0 {
		fmt.eprintln("shader failed to compile")
		return
	}

	quad := make_quad()
	defer rl.UnloadMesh(quad)
	mat := rl.LoadMaterialDefault()
	defer rl.UnloadMaterial(mat)
	mat.shader = shader

	loc_view := rl.GetShaderLocation(shader, "view")
	loc_proj := rl.GetShaderLocation(shader, "proj")
	loc_screen := rl.GetShaderLocation(shader, "screen")
	loc_data_w := rl.GetShaderLocation(shader, "dataWidth")
	loc_order_w := rl.GetShaderLocation(shader, "orderWidth")
	loc_data := rl.GetShaderLocation(shader, "splatData")
	loc_order := rl.GetShaderLocation(shader, "orderTex")

	data_w := i32(DATA_W)
	order_w := i32(ORDER_W)
	rl.SetShaderValue(shader, loc_data_w, &data_w, .INT)
	rl.SetShaderValue(shader, loc_order_w, &order_w, .INT)

	camera: rl.Camera3D
	frame_camera(&camera, view)

	for !rl.WindowShouldClose() {
		if rl.IsFileDropped() {
			files := rl.LoadDroppedFiles()
			if files.count > 0 {
				p := string(files.paths[0])
				next, e := load_view(p)
				if e == .None {
					release_view(&view)
					view = next
					load_err = .None
					frame_camera(&camera, view)
				} else {
					load_err = e
					fmt.eprintf("load %s: %v\n", p, e)
				}
			}
			rl.UnloadDroppedFiles(files)
		}

		if rl.IsKeyPressed(.R) {
			frame_camera(&camera, view)
		}
		rl.UpdateCamera(&camera, .ORBITAL)

		fwd := linalg.normalize(camera.target - camera.position) // import "core:math/linalg"
		if view.count > 0 && (linalg.dot(fwd, last_fwd) < 0.9995 ||
		linalg.length(camera.position - last_pos) > 0.01*view.extent) {
			sort_order(&view, camera.position, fwd)
			rl.UpdateTexture(view.order_tex, raw_data(view.order_px))
			last_fwd, last_pos = fwd, camera.position
		}

		rl.BeginDrawing()
		rl.ClearBackground({16, 14, 20, 255})

		if view.count > 0 {
			aspect := f32(rl.GetScreenWidth()) / f32(max(rl.GetScreenHeight(), 1))
			view_m := rl.GetCameraMatrix(camera)
			proj_m := rl.MatrixPerspective(camera.fovy, aspect, 0.01, max(view.extent*40, 100))
			screen := [2]f32{f32(rl.GetScreenWidth()), f32(rl.GetScreenHeight())}

			rl.SetShaderValueMatrix(shader, loc_view, view_m)
			rl.SetShaderValueMatrix(shader, loc_proj, proj_m)
			rl.SetShaderValue(shader, loc_screen, &screen[0], .VEC2)

			bind_sampler(shader.id, loc_data, view.data_tex.id, 1)
			bind_sampler(shader.id, loc_order, view.order_tex.id, 2)

			rl.BeginMode3D(camera)
			rlgl.DrawRenderBatchActive()
			rlgl.DisableDepthTest()
			rlgl.DisableDepthMask()
			rlgl.DisableBackfaceCulling()
            rlgl.SetBlendMode(i32(rl.BlendMode.CUSTOM))
            rlgl.SetBlendFactors(1, 0x0303, 0x8006) // ONE, ONE_MINUS_SRC_ALPHA, FUNC_ADD
            // rl.DrawMeshInstanced(quad, mat, raw_data(view.xforms), i32(view.count))
			rlgl.EnableShader(shader.id)
			bind_sampler(shader.id, loc_data,  view.data_tex.id,  1)
			bind_sampler(shader.id, loc_order, view.order_tex.id, 2)
			rlgl.EnableVertexArray(quad.vaoId)
			rlgl.DrawVertexArrayElementsInstanced(0, 6, nil, i32(view.visible))
			rlgl.DisableVertexArray()
			rlgl.DisableShader()
            rlgl.SetBlendMode(i32(rl.BlendMode.ALPHA))
			rlgl.EnableDepthTest()
			rlgl.EnableDepthMask()
			rlgl.EnableBackfaceCulling()
			rl.EndMode3D()
		}

		title := view.name if len(view.name) > 0 else path
		rl.DrawText(fmt.ctprintf("%s", title), 16, 14, 20, {236, 230, 220, 255})
		if load_err != .None {
			rl.DrawText(fmt.ctprintf("load failed: %v", load_err), 16, 40, 18, {230, 90, 80, 255})
		} else {
			rl.DrawText(fmt.ctprintf("%d splats   drag orbit   wheel zoom   R reset   drop a .spz or .ply", view.count), 16, 40, 18, {180, 174, 164, 255})
		}
		rl.DrawFPS(16, 68)
		rl.EndDrawing()
	}
}

bind_sampler :: proc(shader_id: u32, loc: i32, tex: u32, slot: i32) {
	if loc < 0 || tex == 0 do return
	rlgl.EnableShader(shader_id)
	rlgl.ActiveTextureSlot(slot)
	rlgl.EnableTexture(tex)

	slot_val := slot
	rlgl.SetUniform(loc, &slot_val, i32(rl.ShaderUniformDataType.INT), 1)
}

// Loads a splat cloud from either an .spz or a 3DGS .ply file. The format is
// detected from the file contents rather than the extension, so renamed or
// extension-less files still work. Both paths hand back a cloud in the same
// coordinate frame (SPZ's native RUB) so the rest of the viewer treats them
// identically.
load_cloud :: proc(path: string) -> (cloud: ospz.Gaussian_Cloud, err: ospz.Spz_Error) {
	data, read_err := os.read_entire_file(path, context.allocator)
	if read_err != nil {
		return {}, .Io_Failed
	}
	defer delete(data)

	if is_ply(data) {
		// 3DGS PLY exports are RDF. Convert to RUB, matching what SPZ decodes to
		// (and what the CLI uses for PLY -> SPZ). Leaving this UNSPECIFIED would
		// skip the conversion and show PLY models rotated 180 degrees about X.
		return ospz.load_splat_from_ply_bytes(data, {to = .RUB})
	}
	// NGSP (v4) and legacy gzip are told apart inside the SPZ loader, which also
	// reports Invalid_Header for anything it doesn't recognise.
	return ospz.load_spz_from_bytes(data, ospz.unpack_options_default())
}

is_ply :: proc(data: []u8) -> bool {
	return len(data) >= 3 && data[0] == 'p' && data[1] == 'l' && data[2] == 'y'
}

load_view :: proc(path: string) -> (view: Splat_View, err: ospz.Spz_Error) {
	cloud, e := load_cloud(path)
	if e != .None {
		return {}, e
	}
	defer ospz.release_gaussian_cloud(&cloud)

	n := int(cloud.num_points)
	if n <= 0 {
		return {}, .Invalid_Data
	}

	pos := make([][3]f32, n)
	texels := n * 4
	data_h := (texels + DATA_W - 1) / DATA_W
	floats := make([]f32, DATA_W*data_h*4)

	min_p := [3]f32{max(f32), max(f32), max(f32)}
	max_p := [3]f32{-max(f32), -max(f32), -max(f32)}
	kept := 0

	for i in 0 ..< n {
        px := cloud.positions[i*3+0]
        py := -cloud.positions[i*3+1]   // flip Y
        pz := cloud.positions[i*3+2]
		if !is_finite(px) || !is_finite(py) || !is_finite(pz) do continue

		sx := math.exp(clamp(cloud.scales[i*3+0], -12, 8))
		sy := math.exp(clamp(cloud.scales[i*3+1], -12, 8))
		sz := math.exp(clamp(cloud.scales[i*3+2], -12, 8))

        // Raw 3DGS PLY quaternions are not unit length (SPZ decodes to unit
        // quaternions already); the shader's quat_mat needs them normalised.
        qx := cloud.rotations[i*4+0]
        qy := cloud.rotations[i*4+1]
        qz := cloud.rotations[i*4+2]
        qw := cloud.rotations[i*4+3]
        qn := math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if is_finite(qn) && qn > 1e-8 {
            inv := 1 / qn
            qx *= inv; qy *= inv; qz *= inv; qw *= inv
        } else {
            qx, qy, qz, qw = 0, 0, 0, 1
        }
        qy = -qy   // conjugate for the Y flip
        qw = -qw

		cr := clamp(0.5+SH_C0*cloud.colors[i*3+0], 0, 1)
		cg := clamp(0.5+SH_C0*cloud.colors[i*3+1], 0, 1)
		cb := clamp(0.5+SH_C0*cloud.colors[i*3+2], 0, 1)
		a := sigmoid(cloud.alphas[i])
		if a < 1.0/255.0 do continue

		base := kept * 16
		floats[base+0] = px
		floats[base+1] = py
		floats[base+2] = pz
		floats[base+3] = a
		floats[base+4] = sx
		floats[base+5] = sy
		floats[base+6] = sz
		floats[base+7] = 0
		floats[base+8] = qx
		floats[base+9] = qy
		floats[base+10] = qz
		floats[base+11] = qw
		floats[base+12] = cr
		floats[base+13] = cg
		floats[base+14] = cb
		floats[base+15] = 1

		pos[kept] = {px, py, pz}
		min_p.x = min(min_p.x, px); min_p.y = min(min_p.y, py); min_p.z = min(min_p.z, pz)
		max_p.x = max(max_p.x, px); max_p.y = max(max_p.y, py); max_p.z = max(max_p.z, pz)
		kept += 1
	}
	if kept == 0 {
		delete(pos)
		delete(floats)
		return {}, .Invalid_Data
	}

	view.count = kept
	view.pos = pos[:kept]
	view.center = (min_p + max_p) * 0.5
	ext := max_p - min_p
	view.extent = max(ext.x, max(ext.y, ext.z))
	if view.extent < 1e-4 do view.extent = 1
	view.name = fmt.aprintf("%s", path)

	view.data_w = DATA_W
	view.data_h = i32(data_h)
	data_img := rl.Image{
		data    = raw_data(floats),
		width   = view.data_w,
		height  = view.data_h,
		mipmaps = 1,
		format  = .UNCOMPRESSED_R32G32B32A32,
	}
	view.data_tex = rl.LoadTextureFromImage(data_img)
	delete(floats)
	rl.SetTextureFilter(view.data_tex, .POINT)
	rl.SetTextureWrap(view.data_tex, .CLAMP)

	view.order = make([]int, kept)
	for i in 0 ..< kept do view.order[i] = i
	view.xforms = make([]rl.Matrix, kept)
	ident := rl.Matrix(1)
	for i in 0 ..< kept do view.xforms[i] = ident

	view.order_h = i32((kept + ORDER_W - 1) / ORDER_W)
	view.order_px = make([]u8, ORDER_W*int(view.order_h)*4)
	for i in 0 ..< kept {
		id := u32(i)
		view.order_px[i*4+0] = u8(id)
		view.order_px[i*4+1] = u8(id >> 8)
		view.order_px[i*4+2] = u8(id >> 16)
		view.order_px[i*4+3] = 255
	}
	order_img := rl.Image{
		data    = raw_data(view.order_px),
		width   = ORDER_W,
		height  = view.order_h,
		mipmaps = 1,
		format  = .UNCOMPRESSED_R8G8B8A8,
	}
	view.order_tex = rl.LoadTextureFromImage(order_img)
	rl.SetTextureFilter(view.order_tex, .POINT)
	rl.SetTextureWrap(view.order_tex, .CLAMP)
	return view, .None
}

release_view :: proc(v: ^Splat_View) {
	if v == nil do return
	if v.data_tex.id != 0 do rl.UnloadTexture(v.data_tex)
	if v.order_tex.id != 0 do rl.UnloadTexture(v.order_tex)
	delete(v.pos)
	delete(v.order)
	delete(v.order_px)
	delete(v.xforms)
	if len(v.name) > 0 do delete(v.name)
	v^ = {}
}

frame_camera :: proc(camera: ^rl.Camera3D, v: Splat_View) {
	dist := v.extent * 1.6
	if dist < 0.5 do dist = 2
	camera^ = {
		position   = v.center + {dist, dist * 0.45, dist},
		target     = v.center,
		up         = {0, 1, 0},
		fovy       = 40,
		projection = .PERSPECTIVE,
	}
}

sort_order :: proc(v: ^Splat_View, cam, fwd: [3]f32) {
	BUCKETS :: 1 << 16
	NEAR :: f32(0.01)
	counts: [BUCKETS]u32 // zero-initialised

	lo, hi := max(f32), -max(f32)
	for p, i in v.pos {
		d := (p.x-cam.x)*fwd.x + (p.y-cam.y)*fwd.y + (p.z-cam.z)*fwd.z
		v.depth[i] = d
		if d > NEAR { lo = min(lo, d); hi = max(hi, d) }
	}
	if hi <= lo { v.visible = 0; return }
	scale := f32(BUCKETS-1) / (hi - lo)

	bucket :: proc(d, lo, scale: f32) -> int {
		// far splats get low buckets, so they are drawn first (back-to-front)
		return (BUCKETS-1) - min(int((d-lo)*scale), BUCKETS-1)
	}
	for d in v.depth do if d > NEAR { counts[bucket(d, lo, scale)] += 1 }

	sum: u32
	for i in 0 ..< BUCKETS { c := counts[i]; counts[i] = sum; sum += c }

	for d, i in v.depth do if d > NEAR {
		b := bucket(d, lo, scale)
		v.order[counts[b]] = u32(i)
		counts[b] += 1
	}
	v.visible = int(sum)
	for k in 0 ..< v.visible {
		id := v.order[k]
		px := v.order_px[k*4:]
		px[0] = u8(id); px[1] = u8(id >> 8); px[2] = u8(id >> 16); px[3] = 255
	}
}

farther :: proc(a, b: int) -> bool {
	pa := g_pos[a]
	pb := g_pos[b]
	da := (pa.x-g_cam.x)*(pa.x-g_cam.x) + (pa.y-g_cam.y)*(pa.y-g_cam.y) + (pa.z-g_cam.z)*(pa.z-g_cam.z)
	db := (pb.x-g_cam.x)*(pb.x-g_cam.x) + (pb.y-g_cam.y)*(pb.y-g_cam.y) + (pb.z-g_cam.z)*(pb.z-g_cam.z)
	return da > db
}

make_quad :: proc() -> rl.Mesh {
	verts   := ([^]f32)(rl.MemAlloc(12 * size_of(f32)))
	uvs     := ([^]f32)(rl.MemAlloc(8  * size_of(f32)))
	indices := ([^]u16)(rl.MemAlloc(6  * size_of(u16)))

	copy(verts[:12],  []f32{-1,-1,0,  1,-1,0,  1,1,0,  -1,1,0})
	copy(uvs[:8],     []f32{0,0, 1,0, 1,1, 0,1})
	copy(indices[:6], []u16{0,1,2, 0,2,3})

	mesh: rl.Mesh
	mesh.vertexCount   = 4
	mesh.triangleCount = 2
	mesh.vertices      = verts
	mesh.texcoords     = uvs
	mesh.indices       = indices
	rl.UploadMesh(&mesh, false)
	return mesh
}

sigmoid :: proc(x: f32) -> f32 {
	if x >= 12 do return 1
	if x <= -12 do return 0
	return 1 / (1 + math.exp(-x))
}

is_finite :: proc(x: f32) -> bool {
	u := transmute(u32)x
	return (u & 0x7fffffff) < 0x7f800000
}

VS :: `#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
uniform mat4 view;
uniform mat4 proj;
uniform vec2 screen;
uniform int dataWidth;
uniform int orderWidth;
uniform sampler2D splatData;
uniform sampler2D orderTex;
flat out vec4 vColor;
flat out vec3 vConic;
out vec2 vPix;

mat3 quat_mat(vec4 q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;
    return mat3(
        1.0-2.0*(yy+zz), 2.0*(xy+wz), 2.0*(xz-wy),
        2.0*(xy-wz),     1.0-2.0*(xx+zz), 2.0*(yz+wx),
        2.0*(xz+wy),     2.0*(yz-wx), 1.0-2.0*(xx+yy)
    );
}

ivec2 at1(int texel, int width) {
    return ivec2(texel - (texel / width) * width, texel / width);
}

void main() {
    vec4 ob = texelFetch(orderTex, at1(gl_InstanceID, orderWidth), 0);
    int id = int(ob.r * 255.0 + 0.5)
           | (int(ob.g * 255.0 + 0.5) << 8)
           | (int(ob.b * 255.0 + 0.5) << 16);
    int base = id * 4;
    vec4 p0 = texelFetch(splatData, at1(base + 0, dataWidth), 0);
    vec4 p1 = texelFetch(splatData, at1(base + 1, dataWidth), 0);
    vec4 p2 = texelFetch(splatData, at1(base + 2, dataWidth), 0);
    vec4 p3 = texelFetch(splatData, at1(base + 3, dataWidth), 0);

    vec3 mean = p0.xyz;
    float opacity = p0.w;
    vec3 scale = max(p1.xyz, vec3(1e-6));
    mat3 R = quat_mat(p2);
    mat3 S = mat3(scale.x, 0.0, 0.0,  0.0, scale.y, 0.0,  0.0, 0.0, scale.z);
    mat3 M = R * S;
    mat3 Sigma = M * transpose(M);

    vec4 tv = view * vec4(mean, 1.0);
    mat3 W = mat3(view);
    mat3 Sv = W * Sigma * transpose(W);

    float fx = proj[0][0] * screen.x * 0.5;
    float fy = proj[1][1] * screen.y * 0.5;
    float tz = tv.z;
    float J00 = fx / tz;
    float J02 = -fx * tv.x / (tz * tz);
    float J11 = fy / tz;
    float J12 = -fy * tv.y / (tz * tz);

    float a = Sv[0][0], b = Sv[0][1], c = Sv[0][2];
    float d = Sv[1][0], e = Sv[1][1], f = Sv[1][2];
    float g = Sv[2][0], h = Sv[2][1], i = Sv[2][2];
    float r0c0 = J00*a + J02*g;
    float r0c1 = J00*b + J02*h;
    float r0c2 = J00*c + J02*i;
    float r1c0 = J11*d + J12*g;
    float r1c1 = J11*e + J12*h;
    float r1c2 = J11*f + J12*i;
    float c00 = r0c0*J00 + r0c2*J02;
    float c01 = r0c1*J11 + r0c2*J12;
    float c11 = r1c1*J11 + r1c2*J12;
    c00 += 0.3;
    c11 += 0.3;
    float det = c00*c11 - c01*c01;
    if (det < 1e-6) det = 1e-6;
    float invDet = 1.0 / det;
    vConic = vec3(c11 * invDet, -c01 * invDet, c00 * invDet);

    float mid = 0.5 * (c00 + c11);
    float disc = max(0.0, mid*mid - det);
    float radius = 3.0 * sqrt(mid + sqrt(disc));
    radius = clamp(radius, 0.5, 1024.0);

    vec2 corner = vertexTexCoord * 2.0 - 1.0;
    vec2 pix = corner * radius;
    vec4 clip = proj * tv;
    if (tz >= -1e-4 || clip.w <= 0.0 || opacity < 1.0/255.0) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        vColor = vec4(0.0);
        vPix = vec2(0.0);
        return;
    }
    clip.xy += (pix / (screen * 0.5)) * clip.w;
    gl_Position = clip;
    vPix = pix;
    vColor = vec4(p3.rgb, opacity);
}
`

FS :: `#version 330
flat in vec4 vColor;
flat in vec3 vConic;
in vec2 vPix;
out vec4 fragColor;

void main() {
    float power = -0.5 * (vConic.x * vPix.x * vPix.x + vConic.z * vPix.y * vPix.y)
                  - vConic.y * vPix.x * vPix.y;
    if (power > 0.0) discard;
    float alpha = exp(power) * vColor.a;
    if (alpha < 1.0/255.0) discard;
    fragColor = vec4(vColor.rgb * alpha, alpha);
}
`