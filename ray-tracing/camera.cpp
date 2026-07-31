#include "raytracing/camera.h"
#include "raytracing/hit.h"
#include "raytracing/material.h"
#include "raytracing/record.h"

namespace RayTracing {

/*vecf3 Camera::ray_color(const Ray& r, int depth, const Hittable& world) const {
    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0)
        return vecf3(0, 0, 0);

    Record rec;

    if (!world.hit(r, Interval(0.001, infinity), rec)) {
        return background;
    }

    // TODO: 4.2 Get the color from scattered or emitted rays
    return background;
}
*/
vecf3 Camera::ray_color(const Ray& r, int depth, const Hittable& world) const {
    // 达到递归深度上限，返回黑色，表示光线能量耗尽
    if (depth <= 0)
        return vecf3(0, 0, 0);

    Record rec;  // 创建记录相交信息的对象

    // 1) 判断 world 是否与 r 相交，如果不相交则返回背景颜色
    if (!world.hit(r, Interval(0.001, infinity), rec)) {
        return background;
    }

    // 2) 直接计算自发光材质的颜色
    vecf3 emitted_color = rec.mat->emitted(rec.u, rec.v, rec.p);  // 获取材质在相交点发出的颜色

    // 3) 判断是否散射，并得到散射光线和衰减颜色
    vecf3 attenuation;  // 衰减颜色
    Ray scattered;  // 散射光线
    if (!rec.mat->scatter(r, rec, attenuation, scattered)) {  // 如果不发生散射
        return emitted_color;  // 返回自发光的颜色
    }

    // 4) 如果发生散射
    // 递归计算下一条光线的颜色，并乘以衰减颜色
    vecf3 scattered_color =  ray_color(scattered, depth - 1, world).cwiseProduct(attenuation);
    //vecf3 scattered_color = ray_color(scattered, depth - 1, world).cwiseQuotient(attenuation);
    // 返回散射颜色与自发光颜色的和
    return scattered_color + emitted_color;
}
}