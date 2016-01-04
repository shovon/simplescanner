# Simple Optical Scanner

This is just me practicing to write a scanner application for camera photos. Instead of using an actual scanner, use this program to extract document content from a photograph, removing all the background, and adjusting the document perspective. Who needs scanners, anyways?

**This is still a work in progress**.

## Usage

```
./simplescanner <path to photograph>
```

## Compiling

You will need

- CMake v2.8
- OpenCV (works on OpenCV v3.1.1, but the code itself should work on v2.4, but I haven't tried)
- C++ 11

Once those are installed, outside the cloned code, create a build folder, and then:

```c++
$ cmake -DCMAKE_CXX_FLAGS=-std=c++11 <path to source folder>
$ make
```

And you should be all good to go.

## Credit

*Automatic perspective correction for quadrilateral objects* -- http://opencv-code.com/tutorials/automatic-perspective-correction-for-quadrilateral-objects/    
*perspective correction for quadrilateral markers* -- http://qtandopencv.blogspot.ca/2013/10/perspective-correction-for.html    
*Canny Edge Detector* -- http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html    
*Answer to: OpenCV C++/Obj-C: Detecting a sheet of paper / Square Detection* -- http://stackoverflow.com/a/8863060/538570    
