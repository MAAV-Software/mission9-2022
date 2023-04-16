//#include <visp/vpImage.h>
#include <visp3/core/vpImage.h>
#include <iostream>

//Visp Stuff
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>

//Other Visp Stuff
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
  try {
    vpImage<unsigned char> gray_image(240, 320);
    vpImage<vpRGBa> color_image(240, 320);

    vpServo task;
 
    gray_image = 128;
    vpRGBa color(255, 0, 0);
    color_image = color;
 
    unsigned int igray_max = gray_image.getHeight() - 1;
    unsigned int jgray_max = gray_image.getWidth() - 1;
    std::cout << "Gray  image, last pixel intensity: " << (int)gray_image[igray_max][jgray_max] << std::endl;
 
    unsigned int icolor_max = color_image.getHeight() - 1;
    unsigned int jcolor_max = color_image.getWidth() - 1;
    std::cout << "Color image, last pixel RGB components: " << (int)color_image[icolor_max][jcolor_max].R << " "
              << (int)color_image[icolor_max][jcolor_max].G << " " << (int)color_image[icolor_max][jcolor_max].B
              << std::endl;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
