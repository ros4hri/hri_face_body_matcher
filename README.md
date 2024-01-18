# hri_face_body_matcher

## Overview

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

## ROS API

### Parameters

All parameters are loaded in the lifecycle `configuration` transition.

- `~confidence_threshold` (default: 0.5):
  Candidate matches with confidence lower that this threshold are not published.

- `~confidence_scaling_factor` (default: 2.0):
  Factor scaling how quickly the estimated confidence drops as the distance between the matched face and body increases.

### Topics

This package follows the ROS4HRI conventions ([REP-155](https://www.ros.org/reps/rep-0155.html)).
If the topic message type is not indicated, the ROS4HRI convention is implied.

#### Subscribed

- `/humans/bodies/tracked`
- `/humans/bodies/<body_id>/skeleton2d`
- `/humans/faces/tracked`
- `/humans/faces/<face_id>/roi`

#### Published

- `/humans/candidate_matches`

### Execution

```bash
ros2 launch hri_face_body_matcher hri_face_body_matcher.launch
```
