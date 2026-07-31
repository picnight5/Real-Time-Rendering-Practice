#include "transform.h"

namespace Utils::Transform {

matf4 perspective(float fov_y, float aspect, float z_near, float z_far) noexcept {
    assert(fov_y > 0 && aspect > 0 && z_near >= 0 && z_far > z_near);

	// TODO 2.1.2 : Create the perspective projection matrix.
	//matf4 m = matf4::Identity();
    matf4 m;
    float tanfov = tan(fov_y * 0.5f);

    m << 1.0f / (tanfov * aspect), 0, 0, 0,
        0, 1.0f / tanfov, 0, 0,
        0, 0, -(1.0f * (z_far + z_near)) / (z_far - z_near), -(2.0f * z_far * z_near) / (z_far - z_near),
        0, 0, -1.0f, 0;
    return m;
}

matf4 orthographic(float width, float height, float z_near, float z_far) noexcept {
    assert(width > 0 && height > 0 && z_near >= 0 && z_far > z_near);

	// TODO 2.1.2 : Create the orthographic projection matrix.
	//matf4 m = matf4::Identity();
    matf4 m;
    m << 2.0f / width, 0, 0, 0,
        0, 2.0f / height, 0, 0,
        0, 0, -2.0f / (z_far - z_near), -(z_far + z_near) / (z_far - z_near),
        0, 0, 0, 1;
    return m;
}

vecf3 cross(const vecf3& v1, const vecf3& v2) {
    return vecf3(
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    );
}
matf4 look_at(const vecf3& pos, const vecf3& target, const vecf3& up) noexcept {
    assert(abs(up.dot(up) - 1.0f) < 1e-6);

    // TODO 2.1.2 : Create the lookat view matrix for the camera.
	//matf4 m = matf4::Identity();
    vecf3 f = target - pos;
    f = f / std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2]* f[2]);
    
    vecf3 r = cross(f, up);
    r = r / std::sqrt(r[0] * r[0] + r[1]* r[1] + r[2] *r[2]);

    vecf3 u = cross(r, f);
    matf4 m;
    m << r[0], r[1], r[2], -r.dot(pos),
        u[0], u[1], u[2], -u.dot(pos),
        -f[0], -f[1], -f[2], f.dot(pos),
        0, 0, 0, 1;
    return m;
}


matf4 get_scale_matrix(const vecf3& scale) noexcept {
    // TODO 2.1.1 : Implement the function that returns a scale matrix.
    //matf4 m = matf4::Identity();

    matf4 m;
    m << scale[0], 0, 0, 0,
        0, scale[1], 0, 0,
        0, 0, scale[2], 0,
        0, 0, 0, 1;
    return m;
}

matf4 get_trans_matrix(const vecf3& trans) noexcept {
    // TODO 2.1.1 : Implement the function that returns a translation matrix.
    //matf4 m = matf4::Identity();

    matf4 m;
    m << 1, 0, 0, trans[0],
        0, 1, 0, trans[1],
        0, 0, 1, trans[2],
        0, 0, 0, 1;
    return m;
}

matf4 rotate_with(float theta, const vecf3& axis) noexcept {
    assert(abs(axis.dot(axis) - 1.0f) < 1e-6);

	// TODO 2.1.1 : Create the rotation matrix with the given theta and axis.
    // Note that the parameter 'theta' is expressed in the radian system.
	//matf4 m = matf4::Identity();

    matf4 m;
    auto c = cos(theta);
    auto s = sin(theta);
    auto t = 1 - c;
    auto u_x = axis[0];
    auto u_y = axis[1]; 
    auto u_z = axis[2];
    m <<
        t * u_x * u_x + c, t* u_x* u_y - s * u_z, t* u_x* u_z + s * u_y, 0,
        t* u_x* u_y + s * u_z, t* u_y* u_y + c, t* u_y* u_z - s * u_x, 0,
        t* u_x* u_z - s * u_y, t* u_y* u_z + s * u_x, t* u_z* u_z + c, 0,
        0, 0, 0, 1;
        
    return m;
}

matf4 generate_model_matrix(const vecf3& pos, const vecf3& scale, float theta, const vecf3& axis) noexcept {
	// TODO 2.1.1 : Implement the function that returns a model matrix.
	matf4 m = matf4::Identity();

	return m;
}

matf4 generate_model_matrix(const vecf3& pos, const vecf3& scale, const matf4& rotate) noexcept {
    matf4 m = get_trans_matrix(pos) * get_scale_matrix(scale) * rotate;

    return m;
}

} // namespace Utils::Transform