
#pragma once
// Jerry Hsu, 2021

#include "vector_type_t.h"

namespace culbvh {


	template<typename T>
	CUDA_INLINE_CALLABLE typename std::conditional<std::is_same<T, float>::value, float3, double3>::type make_point(T v) {
		if constexpr (std::is_same<T, float>::value) {
			return make_float3(v);
		} else {
			return make_double3(v);
		}
	}

    template<typename T>
	CUDA_INLINE_CALLABLE typename std::conditional<std::is_same<T, float>::value, float3, double3>::type make_point(T x, T y, T z) {
		if constexpr (std::is_same<T, float>::value) {
			return make_float3(x, y, z);
		} else {
			return make_double3(x, y, z);
		}
	}

	template <typename T>
	struct Bound {
		using point_t = typename std::conditional<std::is_same<T, float>::value, float3, double3>::type;
		point_t min, max;

		CUDA_INLINE_CALLABLE Bound() : min(make_point<T>(std::numeric_limits<T>::max())), max(make_point<T>(std::numeric_limits<T>::lowest())) {}
		CUDA_INLINE_CALLABLE Bound(const point_t& min, const point_t& max) : min(min), max(max) {}
		CUDA_INLINE_CALLABLE Bound(const point_t& center) : min(center), max(center) {}

		CUDA_INLINE_CALLABLE void pad(T padding) {
			min.x -= padding; min.y -= padding; min.z -= padding;
			max.x += padding; max.y += padding; max.z += padding;
		}

		CUDA_INLINE_CALLABLE void absorb(const point_t& p) {
			min.x = std::min(min.x, p.x);
			min.y = std::min(min.y, p.y);
			min.z = std::min(min.z, p.z);
			max.x = std::max(max.x, p.x);
			max.y = std::max(max.y, p.y);
			max.z = std::max(max.z, p.z);
		}

		CUDA_INLINE_CALLABLE void absorb(const Bound& b) {
			min.x = std::min(min.x, b.min.x);
			min.y = std::min(min.y, b.min.y);
			min.z = std::min(min.z, b.min.z);
			max.x = std::max(max.x, b.max.x);
			max.y = std::max(max.y, b.max.y);
			max.z = std::max(max.z, b.max.z);
		}

		CUDA_INLINE_CALLABLE bool contains(const point_t& p) const {
			return p.x >= min.x && p.x <= max.x &&
				   p.y >= min.y && p.y <= max.y &&
				   p.z >= min.z && p.z <= max.z;
		}

		CUDA_INLINE_CALLABLE bool intersects(const Bound& b) const {
			return !(b.min.x > max.x || b.max.x < min.x ||
					b.min.y > max.y || b.max.y < min.y ||
					b.min.z > max.z || b.max.z < min.z);
		}

		CUDA_INLINE_CALLABLE T surfaceArea() const {
			point_t d = max - min;
			return 2 * (d.x * d.y + d.y * d.z + d.z * d.x);
		}

		CUDA_INLINE_CALLABLE T volume() const {
			point_t d = max - min;
			return d.x * d.y * d.z;
		}

		CUDA_INLINE_CALLABLE point_t center() const {
			return (min + max) * T(0.5);
		}
		CUDA_INLINE_CALLABLE  T width()  const { return max.x - min.x; }
		CUDA_INLINE_CALLABLE  T height() const { return max.y - min.y; }
		CUDA_INLINE_CALLABLE  T depth()  const { return max.z - min.z; }

		CUDA_INLINE_CALLABLE point_t diagonal() const {
			return max - min;
		}

		CUDA_INLINE_CALLABLE point_t normCoord(const point_t& p) const {
			return (p - min) / (max - min);
		}

		CUDA_INLINE_CALLABLE int longestAxis() const {
			point_t d = diagonal();
			if (d.x > d.y && d.x > d.z) return 0;
			if (d.y > d.z) return 1;
			return 2;
		}
	};

	template<typename T>
	struct Bound1D {
		T min;
		T max;

		CUDA_INLINE_CALLABLE Bound1D() :
			min(std::numeric_limits<T>::max()),
			max(std::numeric_limits<T>::lowest()) {
		}
		CUDA_INLINE_CALLABLE Bound1D(T center) : min(center), max(center) {}
		CUDA_INLINE_CALLABLE Bound1D(T min, T max) : min(min), max(max) {}

		CUDA_INLINE_CALLABLE  T center() const {
			return (min + max) * T(0.5);
		}

		CUDA_INLINE_CALLABLE void absorb(const Bound1D<T>& b) {
			min = std::min(min, b.min); max = std::max(max, b.max);
		}

		CUDA_INLINE_CALLABLE void absorb(T b) {
			min = std::min(min, b); max = std::max(max, b);
		}

		CUDA_INLINE_CALLABLE bool contains(T point) const {
			return min <= point && point <= max;
		}

		CUDA_INLINE_CALLABLE bool contains(const Bound1D<T>& b) const {
			return min <= b.min && b.max <= max;
		}

		CUDA_INLINE_CALLABLE bool intersects(const Bound1D<T>& b) const {
			return max > b.min && min < b.max;
		}

		CUDA_INLINE_CALLABLE void pad(T padding) {
			min -= padding; max += padding;
		}

		CUDA_INLINE_CALLABLE T volume() const {
			return max - min;
		}

		CUDA_INLINE_CALLABLE T normCoord(T pos) const {
			return (pos - min) / (max - min);
		}

		CUDA_INLINE_CALLABLE T interp(T coord) const {
			return min + (max - min) * coord;
		}
	};

	typedef Bound1D<float> Range;
     CUDA_INLINE_CALLABLE uint32_t mortonExpandBits(uint32_t v) {
		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	CUDA_INLINE_CALLABLE uint32_t getMorton(const int3& cell) {
		const uint32_t xx = mortonExpandBits(cell.x);
		const uint32_t yy = mortonExpandBits(cell.y);
		const uint32_t zz = mortonExpandBits(cell.z);
		return xx * 4 + yy * 2 + zz;
	}
} // namespace culbvh

  