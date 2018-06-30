#ifndef __LAMBERT_H__
#define __LAMBERT_H__

#include <material.h>

class Lambert : public Material
{
public:
	Lambert(const char* name) : Material(name), Kd_color(0.0f), Kd(1.0f) {}
	Lambert(const char* name, float kd, const Color& kd_color) : Material(name), Kd(kd), Kd_color(kd_color) {}

	float Kd;
	Color Kd_color;

    virtual Spectrum BRDF(const Spectrum& Li, const gmtl::Vec3f& L, const gmtl::Vec3f& V, const IntersectInfo& info) const
    {
		float f = Kd / M_PI;
		return (Li * Kd_color.GetColor(info) * f * gmtl::dot(info.normal, L));
    }
    virtual bool Sample(gmtl::Vec3f& wi, float& pdf, const IntersectInfo& info) const
    {
		wi = info.ray.getDir();
		gmtl::Vec3f wo;

		pdf = this->pdf(wi, wo);

		return true;
    }

    virtual float pdf(const gmtl::Vec3f& wi, const gmtl::Vec3f& wo) const
    {
		return 1.0f / (2 * M_PI);
	}
};


#endif