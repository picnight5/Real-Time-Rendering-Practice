#include "raytracing/hit.h"

namespace RayTracing {


bool HittableList::hit(const Ray& r, Interval ray_t, Record& rec) const {
    Record temp_rec;
    bool hit_anything = false;
    auto closest_so_far = ray_t.max;

    for (const auto& object : objects) {
        if (object->hit(r, Interval(ray_t.min, closest_so_far), temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}


// TODO: 4.3 Sphere & ray intersection
bool Sphere::hit(const Ray& r, Interval ray_t, Record& rec) const {
    vecf3 current_center = center.at(r.time); // 获取球体的中心位置，这里假设球体中心是固定的
    vecf3 oc = current_center - r.origin; // 计算光线原点到球体中心的向量
    auto a = r.direction.squaredNorm(); // 计算光线方向向量的平方长度
    auto h = r.direction.dot(oc); // 计算光线方向向量与 oc 的点积
    auto c = oc.squaredNorm() - radius * radius; // 计算 oc 的平方长度减去球体半径的平方

    auto discriminant = h * h - a * c; // 计算判别式

    // 没有实数根，光线与球体不相交
    if (discriminant < 0) {
        return false;
    }

    auto sqrtd = std::sqrt(discriminant); // 计算判别式的平方根

    // 找到最近、且在光线范围内的交点
    auto root = (h - sqrtd) / a; // 计算第一个交点的 t 值
    if (!ray_t.surrounds(root)) { // 检查第一个交点是否在给定的 t 范围内
        root = (h + sqrtd) / a; // 计算第二个交点的 t 值
        if (!ray_t.surrounds(root)) { // 检查第二个交点是否在给定的 t 范围内
            return false; // 如果两个交点都不在范围内，返回 false
        }
    }

    rec.t = root; // 记录交点的 t 值
    rec.p = r.at(rec.t); // 记录交点的位置

    // 计算法向量
    vecf3 outward_normal = (rec.p - current_center) / radius; // 计算交点处的外法向量
    rec.set_face_normal(r, outward_normal); // 设置法向量，考虑光线是从正面还是背面相交

    // 计算纹理坐标, 用于纹理贴图
    get_sphere_uv(outward_normal, rec.u, rec.v); // 计算纹理坐标 u 和 v

    // 记录材质
    rec.mat = mat; // 记录材质指针

    return true; // 返回 true，表示找到了交点
}


//bool Quad::hit(const Ray& r, Interval ray_t, Record& rec) const {
    

/*class Quad {
public:
    // 四边形的一个角
    Eigen::Vector3f Q;
    // 四边形的两个边向量
    Eigen::Vector3f u, v;
    // 四边形所在平面的法向量
    Eigen::Vector3f n;
    // 平面方程的常数项 D
    float D;
    // 材质指针
    std::shared_ptr<Material> mat;

    Quad(const Eigen::Vector3f& Q, const Eigen::Vector3f& u, const Eigen::Vector3f& v, std::shared_ptr<Material> mat)
        : Q(Q), u(u), v(v), mat(mat) {
        // 计算法向量
        n = u.cross(v).normalized();
        // 计算 D
        D = n.dot(Q);
    }
*/ 
    // TODO: 4.3 Quad & ray intersection
    bool Quad::hit(const Ray& r, Interval ray_t, Record& rec) const {
        // 1) 计算平面与光线的交点
        auto denom = r.direction.dot(normal);
        if (std::abs(denom) < 1e-6) {  // 光线与平面几乎平行
            return false;
        }

        auto t = (D - normal.dot(r.origin)) / denom;
        if (!ray_t.surrounds(t)) {  // 交点不在光线的区间内
            return false;
        }

        vecf3 intersection = r.at(t);

        // 2) 判断交点是否在四边形内部
        vecf3 p = intersection - Q;
        // 计算 alpha 和 beta
        float alpha = w.dot(p.cross(v)); /// n.dot(u.cross(v));
        float beta = w.dot(u.cross(p));// / n.dot(u.cross(v));

        if (alpha < 0.0f || alpha > 1.0f || beta < 0.0f || beta > 1.0f) {
            return false;  // 交点不在四边形内部
        }

        // 3) 记录交点信息
        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }





// Returns the 3D box (six sides) that contains the two opposite vertices a & b.
std::shared_ptr<HittableList> box(const vecf3& a, const vecf3& b, std::shared_ptr<Material> mat) {
    auto sides = std::make_shared<HittableList>();

    // TODO: Use the Quad class to create the six sides of the box.
    // `a` and `b` are the two opposite vertices of the box.
    // 计算立方体的六个面的顶点
    vecf3 p0 = a;
    vecf3 p1 = vecf3(b.x(), a.y(), a.z());
    vecf3 p2 = vecf3(b.x(), b.y(), a.z());
    vecf3 p3 = vecf3(a.x(), b.y(), a.z());
    vecf3 p4 = vecf3(a.x(), a.y(), b.z());
    vecf3 p5 = vecf3(b.x(), a.y(), b.z());
    vecf3 p6 = b;
    vecf3 p7 = vecf3(a.x(), b.y(), b.z());

    // 底面
    sides->add(std::make_shared<Quad>(p0, p1 - p0, p3 - p0, mat));
    // 上面
    sides->add(std::make_shared<Quad>(p5, p4 - p5, p6 - p5, mat));
    // 右面
    sides->add(std::make_shared<Quad>(p0, p3 - p0, p4 - p0, mat));
    // 左面
    sides->add(std::make_shared<Quad>(p1, p5 - p1, p2 - p1, mat));
    // 后面
    sides->add(std::make_shared<Quad>(p3, p2 - p3, p7 - p3, mat));
    // 前面
    sides->add(std::make_shared<Quad>(p0, p4 - p0, p1 - p0, mat));

    return sides;
}


/*bool ConstantMedium::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // TODO: 4.3 Constant Medium & ray intersection
    return false;
}
*/
/*
bool ConstantMedium::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // 1) 计算光线与介质的两个交点
    Record rec1, rec2;
    if (!boundary->hit(r, ray_t, rec1) || !boundary->hit(r, Interval(rec1.t + 0.0001, ray_t.max), rec2)) {
        return false;
    }

    // 2) 计算光线在介质内部的距离
    double distance = rec2.t - rec1.t;
    double random_distance = neg_inv_density * log(random_double());

    // 3) 判断光线是否在介质内部发生了散射
    if (random_distance > distance) {
        return false;
    }

    // 记录交点的t值
    rec.t = rec1.t + random_distance;
    // 记录交点的位置
    rec.p = r.at(rec.t);

    rec.normal = vecf3(1, 0, 0);  // 任意法向量
    rec.front_face = true;      // 任意朝向
    rec.mat = phase_function;

    return true;
}
*/

bool ConstantMedium::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // 1) 计算光线与介质的两个交点
    // 注意取值和大小的判断
    Record rec1, rec2;
    if (!boundary->hit(r, Interval(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()), rec1)) {
        return false;
    }
    if (!boundary->hit(r, Interval(rec1.t + 0.0001, std::numeric_limits<double>::infinity()), rec2)) {
        return false;
    }
    if (rec1.t < ray_t.min) rec1.t = ray_t.min;
    if (rec2.t > ray_t.max) rec2.t = ray_t.max;
    if (rec1.t >= rec2.t) {
        return false;
    }

    // 2) 计算光线在介质内部的距离
    // 利用 random_double() 生成0~1之间的随机数
    double distance_inside_boundary = (rec2.t - rec1.t);
    double hit_distance = neg_inv_density * std::log(random_double());

    // 3) 判断光线是否在介质内部发生了散射
    if (hit_distance > distance_inside_boundary) {
        return false;
    }

    // 记录交点的t值
    // 记录交点的位置
    rec.t = rec1.t + hit_distance;
    rec.p = r.at(rec.t);

    rec.normal = vecf3(1, 0, 0);  // 任意法向量
    rec.front_face = true;      // 任意朝向
    rec.mat = phase_function;

    return true;
}






bool Translate::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // Move the ray backwards by the offset
    Ray offset_r(r.origin - offset, r.direction, r.time);

    // Determine whether an intersection exists along the offset ray (and if so, where)
    if (!object->hit(offset_r, ray_t, rec)) {
        return false;
    }

    // Move the intersection point forwards by the offset
    rec.p += offset;

    return true;
}


bool RotateY::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // Transform the ray from world space to object space.
    auto origin = vecf3(
        (cos_theta * r.origin.x()) - (sin_theta * r.origin.z()),
        r.origin.y(),
        (sin_theta * r.origin.x()) + (cos_theta * r.origin.z())
    );

    auto direction = vecf3(
        (cos_theta * r.direction.x()) - (sin_theta * r.direction.z()),
        r.direction.y(),
        (sin_theta * r.direction.x()) + (cos_theta * r.direction.z())
    );

    Ray rotated_r(origin, direction, r.time);

    // Determine whether an intersection exists in object space (and if so, where).
    if (!object->hit(rotated_r, ray_t, rec))
        return false;

    // Transform the intersection from object space back to world space.
    rec.p = vecf3(
        (cos_theta * rec.p.x()) + (sin_theta * rec.p.z()),
        rec.p.y(),
        (-sin_theta * rec.p.x()) + (cos_theta * rec.p.z())
    );

    rec.normal = vecf3(
        (cos_theta * rec.normal.x()) + (sin_theta * rec.normal.z()),
        rec.normal.y(),
        (-sin_theta * rec.normal.x()) + (cos_theta * rec.normal.z())
    );

    return true;
}


/*bool BVHNode::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // TODO: 4.4 Using BVH to accelerate ray tracing
    return false;
}
*/
//AABB BVHNode::bbox;

bool BVHNode::hit(const Ray& r, Interval ray_t, Record& rec) const {
    // 判断光线与当前节点的AABB是否相交
    if (!bbox.hit(r, ray_t)) {
        return false;
    }

    // 递归地判断与左BVH结点相交
    bool hit_left = left->hit(r, ray_t, rec);

    // 如果左子树有交点，则更新射线参数区间，再递归地判断与右BVH结点相交
    if (hit_left) {
        ray_t.max = rec.t;
    }
    bool hit_right = right->hit(r, ray_t, rec);

    return hit_left || hit_right;
}


}