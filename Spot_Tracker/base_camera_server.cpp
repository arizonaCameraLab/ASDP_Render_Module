#include <math.h>
#include <stdio.h>
#include "base_camera_server.h"

#ifndef	M_PI
#ifndef M_PI_DEFINED
const double M_PI = 2*asin(1.0);
#define M_PI_DEFINED
#endif
#endif

// Sum all the pixels for one color (defaults to the first) in an image.
double image_wrapper_sum(const image_wrapper &img, unsigned rgb)
{
	double sum = 0;
	int minx, maxx, miny, maxy;
	img.read_range(minx, maxx, miny, maxy);
	int x,y;
	for (x = minx; x <= maxx; x++) {
		for (y = miny; y <= maxy; y++) {
			sum += img.read_pixel_nocheck(x, y, rgb);
		}
	}
	return sum;
}

// Sum all the squared values of all pixels for one color
// (defaults to the first) in an image.
double image_wrapper_square_sum(const image_wrapper &img, unsigned rgb)
{
	double sum = 0;
	int minx, maxx, miny, maxy;
	img.read_range(minx, maxx, miny, maxy);
	int x,y;
	for (x = minx; x <= maxx; x++) {
		for (y = miny; y <= maxy; y++) {
			double val = img.read_pixel_nocheck(x, y, rgb);
			sum += val*val;
		}
	}
	return sum;
}

// Apply the specified gain to the value, clamp to zero and the maximum
// possible value, and return the result.  We always clamp to 65535 rather
// than 255 because the high-order byte is used when writing an 8-bit
// file.
static	uint16_t offset_scale_and_clamp(const double value, const double gain, const double offset)
{
  double result = gain * (value + offset);
  if (result < 0) { result = 0; }
  if (result > 65535) { result = 65535; }
  return (uint16_t)(result);
}

bool  image_wrapper::write_to_pgm_file(const char *filename, unsigned channel, double gain, bool sixteen_bits) const
{
  int     r,c;
  int minx, maxx, miny, maxy, numcols, numrows;
  read_range(minx, maxx, miny, maxy);
  numcols = maxx-minx+1;
  numrows = maxy-miny+1;

  if (channel >= get_num_colors()) {
      fprintf(stderr, "image_wrapper::write_to_pgm_file(): Invalid channel)\n");
      return false;
  }

  FILE *f = fopen(filename, "wb");
  if (!f) {
      fprintf(stderr, "image_wrapper::write_to_pgm_file(): Could not open %s for write\n", filename);
      return false;
  }
  int maxval = 255;
  if (sixteen_bits) { maxval = 65535; }
  fprintf(f, "P5\n%d %d\n%d\n", numcols, numrows, maxval);

  // Flip the row values around so that the orientation matches the order expected by PGM files.
  // Make sure to flip on the output, not on the pixel read, because the pixel read
  // may have other hidden transforms in there.
  for (r = 0; r < numrows; r++) {
    for (c = 0; c < numcols; c++) {
      int flip_r = (numrows - 1) - r;
      double  value;
      read_pixel(minx + c, miny + flip_r, value, 0);
      value *= gain;
      if (sixteen_bits) {
        uint16_t uShortValue = static_cast<uint16_t>(value);
        fwrite(&uShortValue, 2, 1, f);
      } else {
        uint8_t uCharValue = static_cast<uint8_t>(value);
        fwrite(&uCharValue, 1, 1, f);
      }
    }
  }

  fclose(f);
  return true;
}

double_image::double_image(int minx, int maxx, int miny, int maxy) :
  _minx(minx), _maxx(maxx), _miny(miny), _maxy(maxy),
  _image(NULL)
{
  // Make sure the parameters are meaningful
  if ( (_minx >= _maxx) || (_miny >= _maxy) ) {
    fprintf(stderr,"double_image::double_image(): Bad min/max coordinates (%d,%d; %d,%d)\n",
      _minx, _miny, _maxx, _maxy);
    _minx = _maxy = _minx = _maxx = 0;
    return;
  }

  // Try to allocate a large enough array to hold all of the values.
  if ( (_image = new double[(_maxx-_minx+1) * (_maxy-_miny+1)]) == NULL) {
    fprintf(stderr,"double_image::double_image(): Out of memory\n");
    _minx = _maxy = _minx = _maxx = 0;
    return;
  }
}

double_image::~double_image()
{
  if (_image != NULL) {
    delete [] _image;
    _image = NULL;
  }
}

void double_image::read_range(int &minx, int &maxx, int &miny, int &maxy) const
{
  minx = _minx; maxx = _maxx; miny = _miny; maxy = _maxy;
};

// Read a pixel from the image into a double; return true if the pixel
// was in the image, false if it was not.
bool double_image::read_pixel(int x, int y, double &result, unsigned /* RGB ignored */) const
{
  int index;
  if (find_index(x,y, index)) {
    result = _image[index];
    return true;
  }
  // Didn't find it, return false.
  return false;
}

float_image::float_image(int minx, int maxx, int miny, int maxy) :
  _minx(minx), _maxx(maxx), _miny(miny), _maxy(maxy),
  _image(NULL)
{
  // Make sure the parameters are meaningful
  if ( (_minx >= _maxx) || (_miny >= _maxy) ) {
    fprintf(stderr,"float_image::float_image(): Bad min/max coordinates (%d,%d; %d,%d)\n",
      _minx, _miny, _maxx, _maxy);
    _minx = _maxy = _minx = _maxx = 0;
    return;
  }

  // Try to allocate a large enough array to hold all of the values.
  if ( (_image = new float[(_maxx-_minx+1) * (_maxy-_miny+1)]) == NULL) {
    fprintf(stderr,"float_image::float_image(): Out of memory\n");
    _minx = _maxy = _minx = _maxx = 0;
    return;
  }
}

float_image::~float_image()
{
  if (_image != NULL) {
    delete [] _image;
    _image = NULL;
  }
}

void float_image::read_range(int &minx, int &maxx, int &miny, int &maxy) const
{
  minx = _minx; maxx = _maxx; miny = _miny; maxy = _maxy;
};

// Read a pixel from the image into a double; return true if the pixel
// was in the image, false if it was not.
bool float_image::read_pixel(int x, int y, double &result, unsigned /* RGB ignored */) const
{
  int index;
  if (find_index(x,y, index)) {
    result = _image[index];
    return true;
  }
  // Didn't find it, return false.
  return false;
}

copy_of_image::copy_of_image(const image_wrapper &copyfrom) :
  _minx(-1), _maxx(-1), _miny(-1), _maxy(-1),
  _numx(-1), _numy(-1), _image(NULL), _numcolors(0)
{
  *this = copyfrom;
}

copy_of_image::copy_of_image(const copy_of_image &copyfrom) :
  _minx(-1), _maxx(-1), _miny(-1), _maxy(-1),
  _numx(-1), _numy(-1), _image(NULL), _numcolors(0)
{
  *this = (const image_wrapper &)copyfrom;
}

void copy_of_image::operator=(const image_wrapper &copyfrom)
{
  // If the dimensions don't match, then get a new image buffer
  int minx, miny, maxx, maxy;
  copyfrom.read_range(minx, maxx, miny, maxy);
  if ( (minx != _minx) || (maxx != _maxx) || (miny != _miny) || (maxy != _maxy) ||
       (get_num_colors() != copyfrom.get_num_colors()) ) {
    if (_image != NULL) { delete [] _image; _image = NULL; }
    _minx = minx; _maxx = maxx; _miny = miny; _maxy = maxy;
    _numx = (_maxx - _minx) + 1;
    _numy = (_maxy - _miny) + 1;
    _numcolors = copyfrom.get_num_colors();
    _image = new double[_numx * _numy * get_num_colors()];
    if (_image == NULL) {
      _numx = _numy = _minx = _maxx = _miny = _maxy = _numcolors = 0;
      return;
    }
  }

  // Copy the values from the image
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	double val;
	copyfrom.read_pixel(x, y, val, c);  // Ignore result outside of image.
	_image[index(x, y, c)] = val;
      }
    }
  }
}

copy_of_image::~copy_of_image()
{
  if (_image) {
    delete [] _image; _image = NULL;
  }
}

bool  copy_of_image::read_pixel(int x, int y, double &result, unsigned rgb) const
{
  if ( (_image == NULL) || (x < _minx) || (x > _maxx) || (y < _miny) || (y > _maxy) ) {
    result = 0.0;
    return false;
  }
  result = _image[index(x, y, rgb)];
  return true;
}

double	copy_of_image::read_pixel_nocheck(int x, int y, unsigned rgb) const
{
  if (_image == NULL) {
    return 0.0;
  }
  return _image[index(x, y, rgb)];
}

subtracted_image::subtracted_image(const image_wrapper &first, const image_wrapper &second, const double offset) :
  _minx(-1), _maxx(-1), _miny(-1), _maxy(-1),
  _numx(-1), _numy(-1), _image(NULL), _numcolors(0)
{
  // Check to make sure that the two images match.
  int minx, miny, maxx, maxy;
  first.read_range(minx, maxx, miny, maxy);
  int minx2, miny2, maxx2, maxy2;
  second.read_range(minx2, maxx2, miny2, maxy2);
  if ( (first.get_num_colors() != second.get_num_colors()) ||
       (minx != minx2) || (miny != miny2) || (maxx != maxx2) || (maxy != maxy2) ) {
    fprintf(stderr,"subtracted_image::subtracted_image(): Two images differ in dimension\n");
    return;
  }

  // Get our image buffer
  _minx = minx; _maxx = maxx; _miny = miny; _maxy = maxy;
  _numx = (_maxx - _minx) + 1;
  _numy = (_maxy - _miny) + 1;
  _numcolors = first.get_num_colors();
  _image = new float[_numx * _numy * get_num_colors()];
  if (_image == NULL) {
    _numx = _numy = _minx = _maxx = _miny = _maxy = _numcolors = 0;
    fprintf(stderr,"subtracted_image::subtracted_image(): Out of memory\n");
    return;
  }

  // Subtract the values from the images, offsetting as we go
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	_image[index(x, y, c)] = static_cast<float>(first.read_pixel_nocheck(x, y, c) - second.read_pixel_nocheck(x, y, c) + offset);
      }
    }
  }
}

subtracted_image::~subtracted_image()
{
  if (_image) {
    delete [] _image; _image = NULL;
  }
}

bool  subtracted_image::read_pixel(int x, int y, double &result, unsigned rgb) const
{
  if ( (_image == NULL) || (x < _minx) || (x > _maxx) || (y < _miny) || (y > _maxy) ) {
    result = 0.0;
    return false;
  }
  result = _image[index(x, y, rgb)];
  return true;
}

double	subtracted_image::read_pixel_nocheck(int x, int y, unsigned rgb) const
{
  if (_image == NULL) {
    return 0.0;
  }
  return _image[index(x, y, rgb)];
}

averaged_image::averaged_image(const image_wrapper &first, const image_wrapper &second) :
  _minx(-1), _maxx(-1), _miny(-1), _maxy(-1),
  _numx(-1), _numy(-1), _image(NULL), _numcolors(0)
{
  // Check to make sure that the two images match.
  int minx, miny, maxx, maxy;
  first.read_range(minx, maxx, miny, maxy);
  int minx2, miny2, maxx2, maxy2;
  second.read_range(minx2, maxx2, miny2, maxy2);
  if ( (first.get_num_colors() != second.get_num_colors()) ||
       (minx != minx2) || (miny != miny2) || (maxx != maxx2) || (maxy != maxy2) ) {
    fprintf(stderr,"averaged_image::averaged_image(): Two images differ in dimension\n");
    return;
  }

  // Get our image buffer
  _minx = minx; _maxx = maxx; _miny = miny; _maxy = maxy;
  _numx = (_maxx - _minx) + 1;
  _numy = (_maxy - _miny) + 1;
  _numcolors = first.get_num_colors();
  _image = new double[_numx * _numy * get_num_colors()];
  if (_image == NULL) {
    _numx = _numy = _minx = _maxx = _miny = _maxy = _numcolors = 0;
    fprintf(stderr,"averaged_image::averaged_image(): Out of memory\n");
    return;
  }

  // Average the values from the images, offsetting as we go
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	_image[index(x, y, c)] = ( first.read_pixel_nocheck(x, y, c) + second.read_pixel_nocheck(x, y, c) ) / 2;
      }
    }
  }
}

averaged_image::~averaged_image()
{
  if (_image) {
    delete [] _image; _image = NULL;
  }
}

bool  averaged_image::read_pixel(int x, int y, double &result, unsigned rgb) const
{
  if ( (_image == NULL) || (x < _minx) || (x > _maxx) || (y < _miny) || (y > _maxy) ) {
    result = 0.0;
    return false;
  }
  result = _image[index(x, y, rgb)];
  return true;
}

double	averaged_image::read_pixel_nocheck(int x, int y, unsigned rgb) const
{
  if (_image == NULL) {
    return 0.0;
  }
  return _image[index(x, y, rgb)];
}

image_metric::image_metric(const image_wrapper &copyfrom) :
  _minx(-1), _maxx(-1), _miny(-1), _maxy(-1),
  _numx(-1), _numy(-1), _image(NULL), _numcolors(0)
{
  // Allocate image buffer
  int minx, miny, maxx, maxy;
  copyfrom.read_range(minx, maxx, miny, maxy);
  _minx = minx; _maxx = maxx; _miny = miny; _maxy = maxy;
  _numx = (_maxx - _minx) + 1;
  _numy = (_maxy - _miny) + 1;
  _numcolors = copyfrom.get_num_colors();
  _image = new double[_numx * _numy * get_num_colors()];
  if (_image == NULL) {
    _numx = _numy = _minx = _maxx = _miny = _maxy = _numcolors = 0;
    fprintf(stderr, "image_metric::image_metric(): Out of memory\n");
    return;
  }

  // Copy the values from the image
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	double val;
	copyfrom.read_pixel(x, y, val, c);  // Ignore result outside of image.
	_image[index(x, y, c)] = val;
      }
    }
  }
}

image_metric::~image_metric()
{
  if (_image) {
    delete [] _image; _image = NULL;
  }
}

bool  image_metric::read_pixel(int x, int y, double &result, unsigned rgb) const
{
  if ( (_image == NULL) || (x < _minx) || (x > _maxx) || (y < _miny) || (y > _maxy) ) {
    result = 0.0;
    return false;
  }
  result = _image[index(x, y, rgb)];
  return true;
}

double	image_metric::read_pixel_nocheck(int x, int y, unsigned rgb) const
{
  if (_image == NULL) {
    return 0.0;
  }
  return _image[index(x, y, rgb)];
}

void minimum_image::operator+=(const image_wrapper &newimage)
{
  // Check to make sure that the two images match.
  int minx, miny, maxx, maxy;
  newimage.read_range(minx, maxx, miny, maxy);
  if ( (newimage.get_num_colors() != get_num_colors()) ||
       (minx != _minx) || (miny != _miny) || (maxx != _maxx) || (maxy != _maxy) ) {
    fprintf(stderr,"minimum_image::+=(): New image differs in dimension\n");
    return;
  }

  // Store the min of the existing and new image
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	double val = read_pixel_nocheck(x, y, c);
	double val2 = newimage.read_pixel_nocheck(x, y, c);
	_image[index(x, y, c)] = (val < val2) ? val : val2;
      }
    }
  }
}

void maximum_image::operator+=(const image_wrapper &newimage)
{
  // Check to make sure that the two images match.
  int minx, miny, maxx, maxy;
  newimage.read_range(minx, maxx, miny, maxy);
  if ( (newimage.get_num_colors() != get_num_colors()) ||
       (minx != _minx) || (miny != _miny) || (maxx != _maxx) || (maxy != _maxy) ) {
    fprintf(stderr,"maximum_image::+=(): New image differs in dimension\n");
    return;
  }

  // Store the max of the existing and new image
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	double val = read_pixel_nocheck(x, y, c);
	double val2 = newimage.read_pixel_nocheck(x, y, c);
        _image[index(x, y, c)] = (val > val2) ? val : val2;
      }
    }
  }
}

void mean_image::operator+=(const image_wrapper &newimage)
{
  // Check to make sure that the two images match.
  int minx, miny, maxx, maxy;
  newimage.read_range(minx, maxx, miny, maxy);
  if ( (newimage.get_num_colors() != get_num_colors()) ||
       (minx != _minx) || (miny != _miny) || (maxx != _maxx) || (maxy != _maxy) ) {
    fprintf(stderr,"mean_image::+=(): New image differs in dimension\n");
    return;
  }

  // Sum the existing image and increase the image count
  int x, y;
  unsigned c;
  for (x = _minx; x <= _maxx; x++) {
    for (y = _miny; y <= _maxy; y++) {
      for (c = 0; c < get_num_colors(); c++) {
	_image[index(x, y, c)] += newimage.read_pixel_nocheck(x, y, c);
      }
    }
  }
  d_num_images++;
}

//< Take the 
bool  mean_image::read_pixel(int x, int y, double &result, unsigned rgb) const
{
  if ( (_image == NULL) || (x < _minx) || (x > _maxx) || (y < _miny) || (y > _maxy) ) {
    result = 0.0;
    return false;
  }
  result = _image[index(x, y, rgb)] / d_num_images;
  return true;
}

double	mean_image::read_pixel_nocheck(int x, int y, unsigned rgb) const
{
  if (_image == NULL) {
    return 0.0;
  }
  return _image[index(x, y, rgb)] / d_num_images;
}
