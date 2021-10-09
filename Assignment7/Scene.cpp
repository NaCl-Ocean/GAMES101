//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"





void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}




Vector3f Scene::shade(const Intersection &hit_point, const Vector3f &w_o, int depth) const
{   
    auto m = hit_point.m;
    // hit the light source
    // without this, the light source will be black
    if(m->hasEmission()){
        return m->getEmission();
    }

    const float epsilon = 0.0005f;

    Vector3f L_dir(0., 0., 0.);
    Vector3f L_indir(0., 0., 0.);

    // Contribution From Light Source

    // randomly sample from the light source
    float pdf_light;
    Intersection hit_light;
    sampleLight(hit_light, pdf_light);

    // calculate the incident direction(the ray from the light source to current position)
    Vector3f point2light_dir = normalize(hit_light.coords - hit_point.coords);
    Ray point2light(hit_point.coords, point2light_dir);


    // check if the ray blocked
    Intersection t = this->intersect(point2light);
    if(std::abs(t.distance - (hit_light.coords - hit_point.coords).norm()) < epsilon){
        // get the brdf
        Vector3f f_r = hit_point.m->eval(-point2light_dir, w_o, hit_point.normal);

        float cos_theta = std::max(0.f, dotProduct(point2light_dir, hit_point.normal));
        float cos_theta_light = std::max(0.f, dotProduct(-point2light_dir, hit_light.normal));

        auto distance = dotProduct((hit_light.coords - hit_point.coords), (hit_light.coords - hit_point.coords));

        L_dir = hit_light.emit * f_r * cos_theta * cos_theta_light / distance;
        L_dir = L_dir / pdf_light;
        // std::cout << "direct light:" << L_dir << std::endl;
    }
    
    // Contribution From Other Objects

    // test Russian Roulette
    float ksi = get_random_float();
    if(ksi <= this->RussianRoulette){
        // randomly sample incident direction from the hemisphere
        Vector3f dir2nextobj = hit_point.m->sample(w_o, hit_point.normal);

        float pdf = hit_point.m->pdf(-w_o, dir2nextobj, hit_point.normal);
        if(pdf > epsilon){
            auto intersect = this->intersect(Ray(hit_point.coords, dir2nextobj));
            auto f_r = hit_point.m->eval(-dir2nextobj, w_o, hit_point.normal);
            float cos_theta = std::max(0.f, dotProduct(dir2nextobj, hit_point.normal));
            if(intersect.happened && !intersect.m->hasEmission()){
                // if pdf is too samll, then the L_indir will explode
                L_indir = this->shade(intersect, -dir2nextobj, depth+1) * f_r * cos_theta / pdf;
                L_indir = L_indir / this->RussianRoulette;
            }
        }
        // std::cout << "indirect light:" << L_indir << std::endl;
    }
    // std::cout << "direct light:" << L_dir << "  indirect light:" << L_indir << std::endl;
    return L_dir + L_indir;

}




// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if(depth > maxDepth){
        return Vector3f(0., 0., 0.);
    }
    Intersection inter = this->intersect(ray);
    if(inter.happened){
        return shade(inter, -ray.direction, depth);
    }else{
        return this->backgroundColor;
    }

}