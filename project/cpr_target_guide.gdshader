shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_never;

// Circular heel target. The transparent centre leaves the chest surface visible.
// Constant green is a target colour, not a correctness measurement.
void fragment() {
	float radius = length(UV - vec2(0.5));
	float distance_to_edge = abs(radius - 0.41) - 0.027;
	float aa = max(fwidth(distance_to_edge), 0.001);
	float ring = 1.0 - smoothstep(-aa, aa, distance_to_edge);
	float centre_dot = 1.0 - smoothstep(0.012, 0.02, radius);
	ALBEDO = vec3(0.24, 1.0, 0.60);
	ALPHA = max(0.95 * ring, 0.65 * centre_dot);
}
