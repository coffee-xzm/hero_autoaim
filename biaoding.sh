ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.028 \
  --k-coefficients 3 \
  --corner-refinement auto \
  --corner-refinement-window 10 \
  --corner-refinement-max-iterations 30 \
  image:=/image_raw \
  camera:=/camera
