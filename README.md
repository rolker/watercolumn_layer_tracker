# watercolumn_layer_tracker

ROS (Noetic) package for tracking sound scattering layers in echosounder
water-column data, written for the Project11 marine robotics framework.
It was developed for the NA155 expedition (E/V Nautilus, October 2023), where
it ran aboard the uncrewed surface vehicle DriX and used the vehicle's EK80
echosounder to keep the midwater robot Mesobot inside a chosen scattering
layer while Mesobot filtered eDNA samples.

The package builds two executables.

| Executable | What it is | Used on NA155 |
|---|---|---|
| `single_layer_tracker` | ROS node. Tracks one operator-selected layer and publishes its depth. | Yes, deployment MB088 |
| `watercolumn_layer_tracker` | Qt desktop tool. Echogram viewer for bag files or a live topic, with an experimental multi-layer detector. | Development and review only |

## The single-layer tracker

`single_layer_tracker` is the algorithm used operationally. The operator
initializes it with the approximate depth of the target layer; from then on
it follows the layer ping by ping.

On each ping the node:

1. takes a 10 m window centered on its current depth estimate;
2. finds the minimum backscatter level in that window and subtracts it, so
   only signal above the local background counts;
3. computes the backscatter-weighted centroid (center of mass) of the
   window;
4. averages the new centroid 50/50 with the previous estimate, so the
   tracked depth follows the layer's migration smoothly while individual
   noisy pings are damped;
5. publishes the result.

Interface:

| | Topic | Type | Notes |
|---|---|---|---|
| Subscribes | `echogram` | `marine_acoustic_msgs/RawSonarImage` | Single-beam water-column pings; intensities as 32-bit floats |
| Subscribes | `~track_layer_at_depth` | `std_msgs/Float32` | Set point. Publishing a depth (re)starts the track there. |
| Publishes | `~layer` | `project11_msgs/AcousticLayer` | One slice per ping; `slices[-1].center_of_mass_range` is the tracked depth |

Range is derived from the ping's sound speed, sample rate and first-sample
index, so the "depth" reported is the range along the beam from the
transducer. For a downward-looking surface-vehicle echosounder this is depth
below the transducer. The 10 m window width is a constant in the source
(`layer_thickness`).

On NA155 the tracked depth was consumed by the `follow_layer` node in
[`mesobot_project11`](https://github.com/rolker/mesobot_project11), which
turned it into depth-target commands for Mesobot over the Sonardyne acoustic
link.

## The Qt tool

`watercolumn_layer_tracker` opens rosbag files (pass them on the command
line or use File > Open) or subscribes to a live echogram topic, draws the
echogram, and runs the `Tracker` class, which segments each ping into slices
above a minimum level and links slices across pings into candidate layers
with configurable minimum and maximum size, depth range and duration. This
multi-layer detector was exploratory and was not used in the field.

## Building

Standard catkin package. Dependencies: `roscpp`, `rosbag`,
`marine_acoustic_msgs`, `project11_msgs`, Qt5 Widgets (for the desktop tool).

## NA155 revision

The tag `na155-mb088` marks the revision of this package that ran on DriX
during Mesobot deployment MB088 on 2023-10-16. The only change since is a
spelling fix in a variable name.

The method is described in section 2.7 of Govindarajan et al., "Verified
Directed Sampling" paper on NA155 midwater eDNA (in preparation; citation to
be updated on publication).

## License

BSD 3-Clause; see [LICENSE](LICENSE). `src/PolynomialRegression.h` is
third-party code by Chris Engelsma under the MIT License.
