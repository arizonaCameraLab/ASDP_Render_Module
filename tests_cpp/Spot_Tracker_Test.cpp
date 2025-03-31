// Copied from https://github.com/CISMM/video and modified to work with the local changes.

#include <chrono>
#include  <math.h>
#include  <stdlib.h>
#include  <stdio.h>
#include  "spot_tracker.h"

#ifdef _WIN32
#define unlink(s) _unlink(s)
#endif

void  compute_disk_chase_statistics(spot_tracker_XY &tracker, double radius, double posaccuracy,
			       int count, double &minerr, double &maxerr, double &sumerr,
			       double &biasx, double &biasy, double &x, double &y)
{
  int i;
  tracker.set_radius(radius);
  tracker.set_pixel_accuracy(posaccuracy);
  tracker.set_location(x,y);
  minerr = 1000; maxerr = 0; sumerr = 0;
  biasx = 0; biasy = 0;
  for (i = 0; i < count; i++) {
    double testx = x + ( (rand()/(double)(RAND_MAX)) - 0.5) * (radius/2);
    double testy = y + ( (rand()/(double)(RAND_MAX)) - 0.5) * (radius/2);
    {
      disc_image image2(0,255, 0,255, 127, 5, testx, testy, radius, 250);
      tracker.optimize_xy(image2, 0, x, y);
      double err = sqrt( (x-testx)*(x-testx) + (y-testy)*(y-testy) );
      if (err < minerr) { minerr = err; }
      if (err > maxerr) { maxerr = err; }
      sumerr += err;
      biasx += x - testx;
      biasy += y - testy;
    }
  }
}

void  compute_cone_chase_statistics(spot_tracker_XY &tracker, double radius, double posaccuracy,
			       int count, double &minerr, double &maxerr, double &sumerr,
			       double &biasx, double &biasy, double &x, double &y)
{
  int i;
  tracker.set_radius(radius);
  tracker.set_pixel_accuracy(posaccuracy);
  tracker.set_location(x,y);
  minerr = 1000; maxerr = 0; sumerr = 0;
  biasx = 0; biasy = 0;
  for (i = 0; i < count; i++) {
    double testx = x + ( (rand()/(double)(RAND_MAX)) - 0.5) * (radius/2);
    double testy = y + ( (rand()/(double)(RAND_MAX)) - 0.5) * (radius/2);
    {
      cone_image image2(0,255, 0,255, 127, 5, testx, testy, radius, 250);
      tracker.optimize_xy(image2, 0, x, y);
      double err = sqrt( (x-testx)*(x-testx) + (y-testy)*(y-testy) );
      if (err < minerr) { minerr = err; }
      if (err > maxerr) { maxerr = err; }
      sumerr += err;
      biasx += x - testx;
      biasy += y - testy;
    }
  }
}

int main(int, char *[])
{
  double  testrad = 5.5, testx = 127.25, testy = 127.75;  //< Actual location of spot
  double  seedrad = 6, seedx = 120, seedy = 118;	  //< Start location for tracking
  int	  avgcount; //< How many samples to average over
  double  pixacc;   //< Pixel accuracy desired
  double  err, minerr, maxerr, sumerr;
  double  raderr, minraderr, maxraderr, sumraderr;
  double  biasx, biasy;

  printf("Generating default test image with radius %lg disk at %lg, %lg\n", testrad, testx, testy);
  disc_image  image(0,255, 0,255, 127, 5, testx, testy, testrad, 250);

  printf("-----------------------------------------------------------------\n");
  printf("Generating default spot tracker\n");
  disk_spot_tracker tracker(seedrad);

  printf("Looking for best fit within the image\n");
  tracker.locate_good_fit_in_image(image, 0, seedx, seedy);

  printf("Optimization, starting at found location %lg, %lg,  rad %lg\n", seedx, seedy, seedrad);
  int i;
  double  x,y, rad, fit;
  tracker.take_single_optimization_step(image, 0, x,y, seedx, seedy);
  for (i = 0; i < 5; i++) {
    tracker.take_single_optimization_step(image, 0, x, y, true, true, true);
    rad = tracker.get_radius();
    fit = tracker.get_fitness();
    printf("Next step: X = %8.3lg,  Y = %8.3lg,  rad = %8.3lg, fit = %12.5lg\n", x,y,rad, fit);
  }

  printf("Chasing around a slightly noisy spot using full optimization\n");
  testrad = 5.5, testx = 127.25, testy = 127.75;  //< Actual location of spot
  seedrad = 6, seedx = 120, seedy = 118;	  //< Start location for tracking
  avgcount = 50;
  minerr = 1000; maxerr = 0; sumerr = 0;
  minraderr = 1000; maxraderr = 0; sumraderr = 0;
  biasx = 0; biasy = 0;
  for (i = 0; i < avgcount; i++) {
    testrad += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * 1;
    if (testrad < 3) { testrad = 3; }
    testx += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    testy += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    {
      disc_image image2(0,255, 0,255, 127, 5, testx, testy, testrad, 250);
      tracker.optimize(image2, 0, x, y);
      rad = tracker.get_radius();
      fit = tracker.get_fitness();
      err = sqrt( (x-testx)*(x-testx) + (y-testy)*(y-testy) );
      if (err < minerr) { minerr = err; }
      if (err > maxerr) { maxerr = err; }
      sumerr += err;
      raderr = fabs(rad-testrad);
      if (raderr < minraderr) { minraderr = raderr; }
      if (raderr > maxraderr) { maxraderr = raderr; }
      sumraderr += raderr;
      biasx += x - testx;
      biasy += y - testy;
      if (i == 0) {
	printf("First opt: real coords (%g,%g), found coords (%g,%g)\n", testx,testy, x,y);
      }
    }
  }
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  printf("Rad err: min=%g, max=%g, mean=%g\n", minraderr, maxraderr, sumraderr/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  avgcount = 50;
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy disk of known radius %g to %g pixel\n", testrad, pixacc);
  compute_disk_chase_statistics(tracker, testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx, biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy cone of known radius %g to %g pixel\n", testrad, pixacc);
  compute_cone_chase_statistics(tracker, testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx, biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  pixacc = 0.05;
  testrad = 5.5;
  tracker.set_pixel_accuracy(pixacc);
  printf("Timing how long it takes to optimize pos to %g pixels from a nearby position on average\n", pixacc);
  avgcount = 1000;
  tracker.optimize(image, 0, x,y);	      // Get back to the correct starting location
  auto start = std::chrono::high_resolution_clock::now();
  for (i = 0; i < avgcount; i++) {
    tracker.optimize_xy(image, 0, x, y,
      x + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2),
      y + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2));
  }
  auto end = std::chrono::high_resolution_clock::now();
  printf("  Time: %lg seconds per optimization\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() / avgcount);

  printf("-----------------------------------------------------------------\n");
  printf("Generating interpolating spot tracker\n");
  disk_spot_tracker_interp interptracker(seedrad);

  printf("Looking for best fit within the image\n");
  interptracker.locate_good_fit_in_image(image, 0, seedx, seedy);

  printf("Optimization, starting at found location %lg, %lg,  rad %lg\n", seedx, seedy, seedrad);
  interptracker.take_single_optimization_step(image, 0, x,y, seedx, seedy);
  for (i = 0; i < 5; i++) {
    interptracker.take_single_optimization_step(image, 0, x, y, true, true, true);
    rad = interptracker.get_radius();
    fit = interptracker.get_fitness();
    printf("Next step: X = %8.3lg,  Y = %8.3lg,  rad = %8.3lg, fit = %12.5lg\n", x,y,rad, fit);
  }

  printf("Chasing around a slightly noisy spot using full optimization\n");
  testrad = 5.5, testx = 127.25, testy = 127.75;  //< Actual location of spot
  seedrad = 6, seedx = 120, seedy = 118;	  //< Start location for tracking
  avgcount = 50;
  minerr = 1000; maxerr = 0; sumerr = 0;
  minraderr = 1000; maxraderr = 0; sumraderr = 0;
  biasx = 0; biasy = 0;
  for (i = 0; i < avgcount; i++) {
    testrad += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * 1;
    if (testrad < 3) { testrad = 3; }
    testx += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    testy += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    {
      disc_image image2(0,255, 0,255, 127, 5, testx, testy, testrad, 250);
      interptracker.optimize(image2, 0, x, y);
      rad = interptracker.get_radius();
      fit = interptracker.get_fitness();
      err = sqrt( (x-testx)*(x-testx) + (y-testy)*(y-testy) );
      if (err < minerr) { minerr = err; }
      if (err > maxerr) { maxerr = err; }
      sumerr += err;
      raderr = fabs(rad-testrad);
      if (raderr < minraderr) { minraderr = raderr; }
      if (raderr > maxraderr) { maxraderr = raderr; }
      sumraderr += raderr;
      biasx += x - testx;
      biasy += y - testy;
      if (i == 0) {
        printf("First opt: real coords (%g,%g), found coords (%g,%g)\n", testx,testy, x,y);
      }
    }
  }
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  printf("Rad err: min=%g, max=%g, mean=%g\n", minraderr, maxraderr, sumraderr/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  avgcount = 50;
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy disk of known radius %g to %g pixel\n", testrad, pixacc);
  compute_disk_chase_statistics(interptracker, testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy cone of known radius %g to %g pixel\n", testrad, pixacc);
  compute_cone_chase_statistics(interptracker, testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  x = 120.5; y = 120;
  disc_image image3(0,255,0,255,127,0,x,y,testrad, 250);
  printf("Optimizing a slightly noisy disk of known radius %g at %g,%g\n", testrad, x,y);
  interptracker.optimize_xy(image3, 0, x, y, floor(x), ceil(y));
  printf("  Found a spot of radius %g at %g,%g\n", interptracker.get_radius(), interptracker.get_x(), interptracker.get_y());

  pixacc = 0.05;
  testrad = 5.5;
  interptracker.set_pixel_accuracy(pixacc);
  printf("Timing how long it takes to optimize pos to %g pixels from a nearby position on average\n", pixacc);
  avgcount = 100;
  interptracker.optimize(image, 0, x,y);	      // Get back to the correct starting location
  start = std::chrono::high_resolution_clock::now();
  for (i = 0; i < avgcount; i++) {
    interptracker.optimize_xy(image, 0, x, y,
      x + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2),
      y + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2));
  }
  end = std::chrono::high_resolution_clock::now();
  printf("  Time: %lg seconds per optimization\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() / avgcount);
  
  printf("-----------------------------------------------------------------\n");
  printf("Generating interpolating cone spot tracker\n");
  cone_spot_tracker_interp conetracker(seedrad);

  printf("Looking for best fit within the image\n");
  conetracker.locate_good_fit_in_image(image, 0, seedx, seedy);

  printf("Optimization, starting at found location %lg, %lg,  rad %lg\n", seedx, seedy, seedrad);
  conetracker.take_single_optimization_step(image, 0, x,y, seedx, seedy);
  for (i = 0; i < 5; i++) {
    conetracker.take_single_optimization_step(image, 0, x, y, true, true, true);
    rad = conetracker.get_radius();
    fit = conetracker.get_fitness();
    printf("Next step: X = %8.3lg,  Y = %8.3lg,  rad = %8.3lg, fit = %12.5lg\n", x,y,rad, fit);
  }

  printf("Chasing around a slightly noisy spot using full optimization\n");
  testrad = 5.5, testx = 127.25, testy = 127.75;  //< Actual location of spot
  seedrad = 6, seedx = 120, seedy = 118;	  //< Start location for tracking
  avgcount = 50;
  minerr = 1000; maxerr = 0; sumerr = 0;
  minraderr = 1000; maxraderr = 0; sumraderr = 0;
  biasx = 0; biasy = 0;
  for (i = 0; i < avgcount; i++) {
    testrad += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * 1;
    if (testrad < 3) { testrad = 3; }
    testx += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    testy += ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2);
    {
      disc_image image2(0,255, 0,255, 127, 5, testx, testy, testrad, 250);
      conetracker.optimize(image2, 0, x, y);
      rad = conetracker.get_radius();
      fit = conetracker.get_fitness();
      err = sqrt( (x-testx)*(x-testx) + (y-testy)*(y-testy) );
      if (err < minerr) { minerr = err; }
      if (err > maxerr) { maxerr = err; }
      sumerr += err;
      raderr = fabs(rad-testrad);
      if (raderr < minraderr) { minraderr = raderr; }
      if (raderr > maxraderr) { maxraderr = raderr; }
      sumraderr += raderr;
      biasx += x - testx;
      biasy += y - testy;
      if (i == 0) {
	printf("First opt: real coords (%g,%g), found coords (%g,%g)\n", testx,testy, x,y);
      }
    }
  }
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  printf("Rad err: min=%g, max=%g, mean=%g\n", minraderr, maxraderr, sumraderr/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  avgcount = 50;
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy disk of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_disk_chase_statistics(conetracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy cone of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_cone_chase_statistics(conetracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  x = 120.5; y = 120;
  disc_image image4(0,255,0,255,127,0,x,y,testrad, 250);
  printf("Optimizing a slightly noisy disk of known radius %g at %g,%g\n", testrad, x,y);
  conetracker.optimize_xy(image4, 0, x, y, floor(x), ceil(y));
  printf("  Found a spot of radius %g at %g,%g\n", conetracker.get_radius(), conetracker.get_x(), conetracker.get_y());

  pixacc = 0.05;
  testrad = 5.5;
  conetracker.set_pixel_accuracy(pixacc);
  printf("Timing how long it takes to optimize pos to %g pixels from a nearby position on average\n", pixacc);
  avgcount = 100;
  conetracker.optimize(image, 0, x,y);	      // Get back to the correct starting location
  start = std::chrono::high_resolution_clock::now();
  for (i = 0; i < avgcount; i++) {
    conetracker.optimize_xy(image, 0, x, y,
      x + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2),
      y + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2));
  }
  end = std::chrono::high_resolution_clock::now();
  printf("  Time: %lg seconds per optimization\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() / avgcount);

  printf("-----------------------------------------------------------------\n");
  printf("Generating interpolating symmetric spot tracker\n");
  seedrad = 6, seedx = 125, seedy = 130;	  //< Start location for tracking
  symmetric_spot_tracker_interp symmetrictracker(seedrad);
  symmetrictracker.set_pixel_accuracy(1);

  printf("Optimization, starting at location %lg, %lg,  locking rad %lg\n", seedx, seedy, seedrad);
  symmetrictracker.take_single_optimization_step(image, 0, x, y, seedx, seedy);
  symmetrictracker.set_radius(seedrad);
  for (i = 0; i < 5; i++) {
    symmetrictracker.take_single_optimization_step(image, 0, x, y, true, true, false);
    rad = symmetrictracker.get_radius();
    fit = symmetrictracker.get_fitness();
    printf("Next step: X = %8.3lg,  Y = %8.3lg,  rad = %8.3lg, fit = %12.5lg\n", x,y,rad, fit);
  }
  testrad = 5.5;
  pixacc = 0.01;
  avgcount = 50;
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy disk of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_disk_chase_statistics(symmetrictracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy cone of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_cone_chase_statistics(symmetrictracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  testrad = 5.5;
  pixacc = 0.05;
  x = 120.5; y = 120;
  disc_image image5(0,255,0,255,127,0,x,y,testrad, 250);
  printf("Optimizing a slightly noisy disk of known radius %g at %g,%g\n", testrad, x,y);
  symmetrictracker.optimize_xy(image5, 0, x, y, floor(x), ceil(y));
  printf("  Found a spot at %g,%g\n", symmetrictracker.get_x(), symmetrictracker.get_y());

  pixacc = 0.05;
  testrad = 5.5;
  symmetrictracker.set_pixel_accuracy(pixacc);
  printf("Timing how long it takes to optimize pos to %g pixels from a nearby position on average\n", pixacc);
  avgcount = 100;
  symmetrictracker.optimize_xy(image, 0, x,y);	      // Get back to the correct starting location
  start = std::chrono::high_resolution_clock::now();
  for (i = 0; i < avgcount; i++) {
    symmetrictracker.optimize_xy(image, 0, x, y,
      x + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2),
      y + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2));
  }
  end = std::chrono::high_resolution_clock::now();
  printf("  Time: %lg seconds per optimization\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() / avgcount);

  printf("-----------------------------------------------------------------\n");
  printf("Generating Gaussian spot tracker\n");

  testrad = 5.5, testx = 127.25, testy = 127.75;  //< Actual location of spot
  seedrad = 6, seedx = 120, seedy = 118;	  //< Start location for tracking
  Gaussian_spot_tracker Gaussiantracker(seedrad, false, 0.25, 0.25, 1.0, 127, 11689);
  printf("Looking for best fit within the image\n");
  Gaussiantracker.locate_good_fit_in_image(image, 0, seedx, seedy);

  printf("Optimization, starting at found location %lg, %lg,  rad %lg\n", seedx, seedy, seedrad);
  Gaussiantracker.take_single_optimization_step(image, 0, x,y, seedx, seedy);
  for (i = 0; i < 5; i++) {
    Gaussiantracker.take_single_optimization_step(image, 0, x, y, true, true, true);
    rad = Gaussiantracker.get_radius();
    fit = Gaussiantracker.get_fitness();
    printf("Next step: X = %8.3lg,  Y = %8.3lg,  rad = %8.3lg, fit = %12.5lg\n", x,y,rad, fit);
  }

  testrad = 5.5;
  pixacc = 0.01;
  avgcount = 50;
  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy disk of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_disk_chase_statistics(Gaussiantracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  x = 120.5; y = 120;
  printf("Chasing around a slightly noisy cone of known radius %g to %g pixel\n", testrad, pixacc);
  // Make the radius slightly larger than the radius of the spot.
  compute_cone_chase_statistics(Gaussiantracker, 1.3*testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx,biasy, x,y);
  printf("Pos err: min=%g, max=%g, mean=%g, xbias = %g, ybias = %g\n", minerr, maxerr, sumerr/avgcount, biasx/avgcount, biasy/avgcount);

  pixacc = 0.05;
  testrad = 5.5;
  Gaussiantracker.set_pixel_accuracy(pixacc);
  printf("Timing how long it takes to optimize pos to %g pixels from a nearby position on average\n", pixacc);
  avgcount = 10;
  Gaussiantracker.optimize_xy(image, 0, x,y);	      // Get back to the correct starting location
  start = std::chrono::high_resolution_clock::now();
  for (i = 0; i < avgcount; i++) {
    Gaussiantracker.optimize(image, 0, x, y,
      x + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2),
      y + ( (rand()/(double)(RAND_MAX)) - 0.5) * 2 * (testrad/2));
  }
  end = std::chrono::high_resolution_clock::now();
  printf("  Time: %lg seconds per optimization\n", std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() / avgcount);

  printf("\nReview the statistics above for the tracker type you want to use.\n");

  printf("\n-----------------------------------------------------------------\n");
  printf("Running quantitative tests on symmetric spot tracker to ensure subpixel accuracy + bias\n");
  {
    seedrad = 6, seedx = 125, seedy = 130;	  //< Start location for tracking
    symmetric_spot_tracker_interp symmetrictracker(seedrad);
    symmetrictracker.set_pixel_accuracy(1);

    testrad = 5.5;
    pixacc = 0.01;
    avgcount = 50;
    x = testx; y = testy;
    // Make the radius slightly larger than the radius of the spot.
    compute_disk_chase_statistics(symmetrictracker, 1.3 * testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx, biasy, x, y);
    if (maxerr > 0.5) {
      fprintf(stderr, "ERROR: Maximum disc error is greater than 0.5 pixels: %lg\n", maxerr);
      return 1;
    }
    if (fabs(biasx / avgcount) > 0.1 || fabs(biasy / avgcount) > 0.1) {
      fprintf(stderr, "ERROR: Bias for disc is greater than 0.1 pixels: %lg, %lg\n", biasx / avgcount, biasy / avgcount);
      return 2;
    }

    x = testx; y = testy;
    // Make the radius slightly larger than the radius of the spot.
    compute_cone_chase_statistics(symmetrictracker, 1.3 * testrad, pixacc, avgcount, minerr, maxerr, sumerr, biasx, biasy, x, y);
    if (maxerr > 0.1) {
      fprintf(stderr, "ERROR: Maximum cone error is greater than 0.1 pixels: %lg\n", maxerr);
      return 10;
    }
    if (fabs(biasx / avgcount) > 0.1 || fabs(biasy / avgcount) > 0.1) {
      fprintf(stderr, "ERROR: Bias for cone is greater than 0.1 pixels: %lg, %lg\n", biasx / avgcount, biasy / avgcount);
      return 11;
    }
  }

  printf("\nSuccess!\n");
  return 0;
}
