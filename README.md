hri_face_body_matcher
=====================

Overview
--------

`hri_face_body_matcher` is a [ROS4HRI](https://wiki.ros.org/hri)-compatible face
to body matcher node.

It finds the most likely matches between the recognized faces and bodies based
on their relative position in the source image.

### Algorithm

For each of the possible associations of recognized face and body, a matching
cost is computed, linearly decreasing with the distance between the body nose
and the face center in the image.

The rate the confidence drops is proportional to the
`~confidence_scaling_factor` parameter and the face size, intended as its
diagonal length.  $$confidence = max(0, 1 - \frac{distance * c.s.f.}{2 * face\
size})$$

ROS API
-------

### Parameters

- `~confidence_threshold` (double ∈ [0,1], default: 0.5): candidate matches with
  confidence lower that this threshold are not published.

- `~confidence_scaling_factor` (double > 0, default: 2): factor scaling how
  quickly the estimated confidence drops as the distance between the matched
  face and body increases.

### Topics

`hri_face_body_matcher` follows the ROS4HRI conventions
([REP-155](https://www.ros.org/reps/rep-0155.html)).

#### Subscribed topics

The node uses [libhri](https://gitlab/ros4hri/libhri) to indirectly subscribe to
all ROS4HRI topics.

The required ROS4HRI topics used by the package are:

- `/humans/bodies/tracked`
- `/humans/bodies/<body_id>/skeleton2d`
- `/humans/faces/tracked`
- `/humans/faces/<face_id>/roi`

#### Published topics

- `/humans/candidate_matches`
  ([hri_msgs/IdsMatch](http://docs.ros.org/en/api/hri_msgs/html/msg/IdsMatch.html)):
  correspondences between face IDs and body IDs (alongside with a confidence
  level).

### Execution

```bash
roslaunch hri_face_body_matcher hri_face_body_matcher.launch
```
