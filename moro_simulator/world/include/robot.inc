define laserscanner ranger
(
  sensor
  (
    range [ 0.0 5.0 ]
    fov 160.0
    samples 270
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define beaconreceiver fiducial
(
  range_min 0.0
  range_max 10.0
  range_max_id 10.0
  fov 160.0
  ignore_zloc 1

  size [ 0.1 0.1 0.1 ]
  color "green"
)

define robot position
(
  drive "diff"
  velocity [ 0 0 0 0 ]

  localization "odom"
  odom_error [ 0.03 0.03 0 0.05 ]

  velocity_bounds [ -0.5 0.5 0 0 0 0 -45 45 ]
  acceleration_bounds [ -1 1 0 0 0 0 -45 45 ]

  color "gray"
  size [ 0.2552 0.2552 0.4 ]
  origin [ 0 0 0 0 ]
  gui_nose 1

  laserscanner(pose [ -0.1 0 -0.11 0 ])
  beaconreceiver(pose [ -0.1 0 -0.11 0 ])
)
