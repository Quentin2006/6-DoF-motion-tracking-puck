
#pragma once
#include <algorithm>
#include <glm/glm.hpp>
#include <vector>
namespace spline {

// Evaluates the tangent (first derivative) of a Catmull-Rom spline at parameter
// t. Input: Four control points p0-p3 defining the spline segment, and
// parameter t (typically in [0, 1]). Output: A glm::vec3 representing the
// tangent (direction vector) at the point on the curve corresponding to t.
inline glm::vec3 catmullRomTangent(const glm::vec3 &p0, const glm::vec3 &p1,
                                   const glm::vec3 &p2, const glm::vec3 &p3,
                                   float t) {
  float t2 = t * t;
  return 0.5f *
         ((-p0 + p2) + (4.0f * p0 - 10.0f * p1 + 8.0f * p2 - 2.0f * p3) * t +
          (-3.0f * p0 + 9.0f * p1 - 9.0f * p2 + 3.0f * p3) * t2);
}

// Evaluates a Catmull-Rom spline at parameter t for four control points.
// Input: Four control points p0-p3 defining the spline segment, and parameter t
// (in [0, 1]).
//        The curve passes through p1 (at t=0) and p2 (at t=1).
// Output: A glm::vec3 representing the point on the spline curve at parameter
// t.
inline glm::vec3 catmullRom(const glm::vec3 &p0, const glm::vec3 &p1,
                            const glm::vec3 &p2, const glm::vec3 &p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5f * ((2.0f * p1) + (-p0 + p2) * t +
                 (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                 (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

// Subdivides a set of control points into a smooth Catmull-Rom spline with the
// specified number of samples. Input: A vector of glm::vec3 control points, the
// desired number of samples (totalSamples),
//        and an optional cyclic flag (if true, the spline wraps around;
//        first/last point should match).
// Output: A vector of glm::vec3 points representing the smooth interpolated
// spline curve.
inline std::vector<glm::vec3> subdivide(const std::vector<glm::vec3> &points,
                                        int totalSamples, bool cyclic = false) {
  std::vector<glm::vec3> result;
  result.reserve(totalSamples + 1);

  if (points.size() < 2)
    return points;

  int numSegments = (int)points.size() - 1;

  for (int i = 0; i < totalSamples; ++i) {
    float globalT = (float)i / (float)totalSamples * (float)numSegments;
    int seg = (int)globalT;
    float localT = globalT - (float)seg;

    // Clamp segment index
    if (seg >= numSegments) {
      seg = numSegments - 1;
      localT = 1.0f;
    }

    // Get the 4 control points for this segment
    glm::vec3 p0, p1, p2, p3;
    if (cyclic) {
      int n = numSegments; // number of unique points (last == first)
      p0 = points[((seg - 1) % n + n) % n];
      p1 = points[seg % n];
      p2 = points[(seg + 1) % n];
      p3 = points[(seg + 2) % n];
    } else {
      int n = (int)points.size();
      p0 = points[std::max(seg - 1, 0)];
      p1 = points[seg];
      p2 = points[std::min(seg + 1, n - 1)];
      p3 = points[std::min(seg + 2, n - 1)];
    }

    result.push_back(catmullRom(p0, p1, p2, p3, localT));
  }

  // For non-cyclic paths, ensure we reach the exact endpoint
  if (!cyclic && totalSamples > 0) {
    result.push_back(points.back());
  }

  // For cyclic paths, close the loop
  if (cyclic) {
    result.push_back(result.front());
  }

  return result;
}

// reutrn tuple of position and tangent
inline std::tuple<glm::vec3, glm::vec3>
calculatePosOnSpline(bool cyclic, int numSegments,
                     const std::vector<glm::vec3> &pts, float globalT) {

  // Determine which segment we're on and fractional position within it
  int seg = (int)globalT;
  float localT = globalT - (float)seg;

  // Clamp to last segment if we've exceeded the path
  if (seg >= numSegments) {
    seg = numSegments - 1;
    localT = 1.0f;
  }
  // Get the four control points for Catmull-Rom interpolation
  // p0 = previous point, p1 = start of current segment
  // p2 = end of current segment, p3 = next point
  glm::vec3 p0, p1, p2, p3;
  if (cyclic) {
    // For cyclic paths, wrap indices using modulo
    int n = numSegments;
    p0 = pts[((seg - 1) % n + n) % n];
    p1 = pts[seg % n];
    p2 = pts[(seg + 1) % n];
    p3 = pts[(seg + 2) % n];
  } else {
    // For non-cyclic paths, clamp to path endpoints
    int n = (int)pts.size();
    p0 = pts[std::max(seg - 1, 0)];
    p1 = pts[seg];
    p2 = pts[std::min(seg + 1, n - 1)];
    p3 = pts[std::min(seg + 2, n - 1)];
  }

  // Compute tangent for orientation (face direction of travel)
  glm::vec3 tangent = spline::catmullRomTangent(p0, p1, p2, p3, localT);
  tangent = glm::normalize(tangent);

  glm::vec3 pos = spline::catmullRom(p0, p1, p2, p3, localT);

  return std::make_tuple(pos, tangent);
}

} // namespace spline
