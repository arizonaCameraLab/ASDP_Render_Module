#include <math.h>   // for floor, fmod
#include <stddef.h> // for size_t
#include <stdio.h>  // for fprintf() and such
#include <ctime>

#ifdef _MSC_VER
// Don't tell us about strcpy being dangerous.
#define _CRT_SECURE_NO_WARNINGS 1
#pragma warning(disable : 4996)
#endif

#include "vrpn_Shared.h"

#ifdef _WIN32
#ifdef _MSC_VER
#pragma comment(lib, "wsock32.lib") // VRPN requires the Windows Sockets library.
#endif
#endif

#if defined(__APPLE__) || defined(__ANDROID__)
#include <unistd.h>
#endif

#if !(defined(_WIN32) && defined(VRPN_USE_WINSOCK_SOCKETS))
#include <sys/select.h> // for select
#include <netinet/in.h> // for htonl, htons
#endif

#define CHECK(a) \
    if (a == -1) return -1

#if defined(VRPN_USE_WINSOCK_SOCKETS)
/* from HP-UX */\
/* minGW guards the definition, so we do so here as well.
minGW declares more, so we are not defining _TIMEZONE_DEFINED. */
#ifndef _TIMEZONE_DEFINED /* also in sys/time.h */
struct timezone {
	int tz_minuteswest; /* minutes west of Greenwich */
	int tz_dsttime;     /* type of dst correction */
};
#endif /* _TIMEZONE_DEFINED */
#endif

// perform normalization of a timeval
// XXX this still needs to be checked for errors if the timeval
// or the rate is negative
static inline void timevalNormalizeInPlace(timeval &in_tv)
{
    const long div_77777 = (in_tv.tv_usec / 1000000);
    in_tv.tv_sec += div_77777;
    in_tv.tv_usec -= (div_77777 * 1000000);
}
timeval vrpn_TimevalNormalize(const timeval &in_tv)
{
    timeval out_tv = in_tv;
    timevalNormalizeInPlace(out_tv);
    return out_tv;
}

// Calcs the sum of tv1 and tv2.  Returns the sum in a timeval struct.
// Calcs negative times properly, with the appropriate sign on both tv_sec
// and tv_usec (these signs will match unless one of them is 0).
// NOTE: both abs(tv_usec)'s must be < 1000000 (ie, normal timeval format)
timeval vrpn_TimevalSum(const timeval &tv1, const timeval &tv2)
{
    timeval tvSum = tv1;

    tvSum.tv_sec += tv2.tv_sec;
    tvSum.tv_usec += tv2.tv_usec;

    // do borrows, etc to get the time the way i want it: both signs the same,
    // and abs(usec) less than 1e6
    if (tvSum.tv_sec > 0) {
        if (tvSum.tv_usec < 0) {
            tvSum.tv_sec--;
            tvSum.tv_usec += 1000000;
        }
        else if (tvSum.tv_usec >= 1000000) {
            tvSum.tv_sec++;
            tvSum.tv_usec -= 1000000;
        }
    }
    else if (tvSum.tv_sec < 0) {
        if (tvSum.tv_usec > 0) {
            tvSum.tv_sec++;
            tvSum.tv_usec -= 1000000;
        }
        else if (tvSum.tv_usec <= -1000000) {
            tvSum.tv_sec--;
            tvSum.tv_usec += 1000000;
        }
    }
    else {
        // == 0, so just adjust usec
        if (tvSum.tv_usec >= 1000000) {
            tvSum.tv_sec++;
            tvSum.tv_usec -= 1000000;
        }
        else if (tvSum.tv_usec <= -1000000) {
            tvSum.tv_sec--;
            tvSum.tv_usec += 1000000;
        }
    }

    return tvSum;
}

// Calcs the diff between tv1 and tv2.  Returns the diff in a timeval struct.
// Calcs negative times properly, with the appropriate sign on both tv_sec
// and tv_usec (these signs will match unless one of them is 0)
timeval vrpn_TimevalDiff(const timeval &tv1, const timeval &tv2)
{
    timeval tv;

    tv.tv_sec = -tv2.tv_sec;
    tv.tv_usec = -tv2.tv_usec;

    return vrpn_TimevalSum(tv1, tv);
}

timeval vrpn_TimevalScale(const timeval &tv, double scale)
{
    timeval result;
    result.tv_sec = (long)(tv.tv_sec * scale);
    result.tv_usec =
        (long)(tv.tv_usec * scale + fmod(tv.tv_sec * scale, 1.0) * 1000000.0);
    timevalNormalizeInPlace(result);
    return result;
}

// returns 1 if tv1 is greater than tv2;  0 otherwise
bool vrpn_TimevalGreater(const timeval &tv1, const timeval &tv2)
{
    if (tv1.tv_sec > tv2.tv_sec) return 1;
    if ((tv1.tv_sec == tv2.tv_sec) && (tv1.tv_usec > tv2.tv_usec)) return 1;
    return 0;
}

// return 1 if tv1 is equal to tv2; 0 otherwise
bool vrpn_TimevalEqual(const timeval &tv1, const timeval &tv2)
{
    if (tv1.tv_sec == tv2.tv_sec && tv1.tv_usec == tv2.tv_usec)
        return true;
    else
        return false;
}

unsigned long vrpn_TimevalDuration(struct timeval endT, struct timeval startT)
{
    return (endT.tv_usec - startT.tv_usec) +
           1000000L * (endT.tv_sec - startT.tv_sec);
}

double vrpn_TimevalDurationSeconds(struct timeval endT, struct timeval startT)
{
    return (endT.tv_usec - startT.tv_usec) / 1000000.0 +
           (endT.tv_sec - startT.tv_sec);
}

double vrpn_TimevalMsecs(const timeval &tv)
{
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

timeval vrpn_MsecsTimeval(const double dMsecs)
{
    timeval tv;
    tv.tv_sec = (long)floor(dMsecs / 1000.0);
    tv.tv_usec = (long)((dMsecs / 1000.0 - tv.tv_sec) * 1e6);
    return tv;
}

// Sleep for dMsecs milliseconds, freeing up the processor while you
// are doing so.

void vrpn_SleepMsecs(double dMilliSecs)
{
#if defined(_WIN32)
    Sleep((DWORD)dMilliSecs);
#else
    timeval timeout;

    // Convert milliseconds to seconds
    timeout.tv_sec = (int)(dMilliSecs / 1000.0);

    // Subtract of whole number of seconds
    dMilliSecs -= timeout.tv_sec * 1000;

    // Convert remaining milliseconds to microsec
    timeout.tv_usec = (int)(dMilliSecs * 1000);

    // A select() with NULL file descriptors acts like a microsecond
    // timer.
    select(0, 0, 0, 0, &timeout); // wait for that long;
#endif
}

// convert vrpn_float64 to/from network order
// I have chosen big endian as the network order for vrpn_float64
// to match the standard for htons() and htonl().
// NOTE: There is an added complexity when we are using an ARM
// processor in mixed-endian mode for the doubles, whereby we need
// to not just swap all of the bytes but also swap the two 4-byte
// words to get things in the right order.
#if defined(__arm__)
#include <endian.h>
#endif

vrpn_float64 vrpn_htond(vrpn_float64 d)
{
    if (!vrpn_big_endian) {
        vrpn_float64 dSwapped;
        char *pchSwapped = (char *)&dSwapped;
        char *pchOrig = (char *)&d;

        // swap to big-endian order.
        unsigned i;
        for (i = 0; i < sizeof(vrpn_float64); i++) {
            pchSwapped[i] = pchOrig[sizeof(vrpn_float64) - i - 1];
        }

#if defined(__arm__) && !defined(__ANDROID__)
// On ARM processor, see if we're in mixed mode.  If so,
// we need to swap the two words after doing the total
// swap of bytes.
#if __FLOAT_WORD_ORDER != __BYTE_ORDER
        {
            /* Fixup mixed endian floating point machines */
            vrpn_uint32 *pwSwapped = (vrpn_uint32 *)&dSwapped;
            vrpn_uint32 scratch = pwSwapped[0];
            pwSwapped[0] = pwSwapped[1];
            pwSwapped[1] = scratch;
        }
#endif
#endif

        return dSwapped;
    }
    else {
        return d;
    }
}

// they are their own inverses, so ...
vrpn_float64 vrpn_ntohd(vrpn_float64 d) { return vrpn_htond(d); }

/** Utility routine for placing a timeval struct into a buffer that
    is to be sent as a message. Handles packing into an unaligned
    buffer (though this should not be done). Advances the insertPt
    pointer to just after newly-inserted value. Decreases the buflen
    (space remaining) by the length of the value. Returns zero on
    success and -1 on failure.

    Part of a family of routines that buffer different VRPN types
    based on their type (vrpn_buffer is overloaded based on the third
    parameter type). These routines handle byte-swapping to the
    VRPN standard wire protocol.
*/

int vrpn_buffer(char **insertPt, vrpn_int32 *buflen, const timeval t)
{
    vrpn_int32 sec, usec;

    // tv_sec and usec are 64 bits on some architectures, but we
    // define them as 32 bit for network transmission

    sec = t.tv_sec;
    usec = t.tv_usec;

    if (vrpn_buffer(insertPt, buflen, sec)) return -1;
    return vrpn_buffer(insertPt, buflen, usec);
}

/** Utility routine for placing a character string of given length
    into a buffer that is to be sent as a message. Handles packing into
    an unaligned buffer (though this should not be done). Advances the insertPt
    pointer to just after newly-inserted value. Decreases the buflen
    (space remaining) by the length of the value. Returns zero on
    success and -1 on failure.

    Part of a family of routines that buffer different VRPN types
    based on their type (vrpn_buffer is overloaded based on the third
    parameter type). These routines handle byte-swapping to the
    VRPN standard wire protocol.

    If the length is specified as -1, then the string will be assumed to
    be NULL-terminated and will be copied using the string-copy routines.
*/

int vrpn_buffer(char **insertPt, vrpn_int32 *buflen,
                         const char *string, vrpn_int32 length)
{
    if (length > *buflen) {
        fprintf(stderr, "vrpn_buffer:  buffer not long enough for string.\n");
        return -1;
    }

    if (length == -1) {
        size_t len =
            strlen(string) + 1; // +1 for the NULL terminating character
        if (len > (unsigned)*buflen) {
            fprintf(stderr,
                    "vrpn_buffer:  buffer not long enough for string.\n");
            return -1;
        }
        vrpn_strncpynull(*insertPt, string, len);
        *insertPt += len;
        *buflen -= static_cast<vrpn_int32>(len);
    }
    else {
        memcpy(*insertPt, string, length);
        *insertPt += length;
        *buflen -= length;
    }

    return 0;
}

/** Utility routine for taking a struct timeval from a buffer that
    was sent as a message. Handles unpacking from an
    unaligned buffer, because people did this anyway. Advances the reading
    pointer to just after newly-read value. Assumes that the
    buffer holds a complete value. Returns zero on success and -1 on failure.

    Part of a family of routines that unbuffer different VRPN types
    based on their type (vrpn_buffer is overloaded based on the third
    parameter type). These routines handle byte-swapping to and from
    the VRPN defined wire protocol.
*/

int vrpn_unbuffer(const char **buffer, timeval *t)
{
    vrpn_int32 sec, usec;

    CHECK(vrpn_unbuffer(buffer, &sec));
    CHECK(vrpn_unbuffer(buffer, &usec));

    t->tv_sec = sec;
    t->tv_usec = usec;

    return 0;
}

/** Utility routine for taking a string of specified length from a buffer that
    was sent as a message. Does NOT handle unpacking from an
    unaligned buffer, because the semantics of VRPN require
    message buffers and the values in them to be aligned, in order to
    reduce the amount of copying that goes on. Advances the read
    pointer to just after newly-read value. Assumes that the
    buffer holds a complete value. Returns zero on success and -1 on failure.

    Part of a family of routines that unbuffer different VRPN types
    based on their type (vrpn_buffer is overloaded based on the third
    parameter type). These routines handle byte-swapping to and from
    the VRPN defined wire protocol.

    If the length is specified as less than zero, then the string will be
   assumed to be NULL-terminated and will be read using the string-copy
   routines with a length that is at most the magnitude of the number
   (-16 means at most 16).
     NEVER use this on a string that was packed with other than the
   NULL-terminating condition, since embedded NULL characters will ruin the
   argument parsing for any later arguments in the message.
*/

int vrpn_unbuffer(const char **buffer, char *string, vrpn_int32 length)
{
    if (!string) return -1;

    if (length < 0) {
        // Read the string up to maximum length, then check to make sure we
        // found the null-terminator in the length we read.
        size_t max_len = static_cast<size_t>(-length);
        strncpy(string, *buffer, max_len);
        size_t i;
        bool found = false;
        for (i = 0; i < max_len; i++) {
            if (string[i] == '\0') {
                found = true;
                break;
            }
        }
        if (!found) {
            return -1;
        }
        *buffer += strlen(*buffer) + 1; // +1 for NULL terminating character
    } else {
        memcpy(string, *buffer, length);
        *buffer += length;
    }

    return 0;
}

//=====================================================================
// This section contains various implementations of vrpn_gettimeofday().
//   Which one is selected depends on various #defines.  There is a second
// section that deals with handling various configurations on Windows.
//   The first section deals with the fact that we may want to use the
// std::chrono classes introduced in C++-11 as a cross-platform (even
// Windows) solution to timing.  If VRPN_USE_STD_CHRONO is defined, then
// we do this -- converting from chrono epoch and interval into the
// gettimeofday() standard tick of microseconds and epoch start of
// midnight, January 1, 1970.

///////////////////////////////////////////////////////////////
// Implementation with std::chrono follows, and overrides any of
// the Windows-specific definitions.
///////////////////////////////////////////////////////////////

#include <chrono>
#include <mutex>
#include <atomic>

///////////////////////////////////////////////////////////////
// With Visual Studio 2013 64-bit, the hires clock produces a clock that has a
// tick interval of around 15.6 MILLIseconds, repeating the same
// time between them.
///////////////////////////////////////////////////////////////
// With Visual Studio 2015 64-bit, the hires clock produces a good, high-
// resolution clock with no blips.  However, its epoch seems to
// restart when the machine boots, whereas the system clock epoch
// starts at the standard midnight January 1, 1970.
///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
// Helper function to convert from the high-resolution clock
// time to the equivalent system clock time (assuming no clock
// adjustment on the system clock since program start).
//  To make this thread safe, we semaphore the determination of
// the offset to be applied.  To handle a slow-ticking system
// clock, we repeatedly sample it until we get a change.
//  This assumes that the high-resolution clock on different
// threads has the same epoch.
///////////////////////////////////////////////////////////////

static std::atomic_bool hr_offset_determined = false;
static std::mutex hr_offset_semaphore;
static struct timeval hr_offset;

static struct timeval high_resolution_time_to_system_time(
    struct timeval hi_res_time //< Time computed from high-resolution clock
    )
{
    // If we haven't yet determined the offset between the high-resolution
    // clock and the system clock, do so now.  Avoid a race between threads
    // using the semaphore and checking the boolean both before and after
    // grabbing the semaphore (in case someone beat us to it).
    if (!hr_offset_determined) {
        hr_offset_semaphore.lock();
        // Someone else who had the semaphore may have beaten us to this.
        if (!hr_offset_determined) {
            // Watch the system clock until it changes; this will put us
            // at a tick boundary.  On many systems, this will change right
            // away, but on Windows 8 it will only tick every 16ms or so.
            std::chrono::system_clock::time_point pre =
                std::chrono::system_clock::now();
            std::chrono::system_clock::time_point post;
            // On Windows 8.1, this took from 1-16 ticks, and seemed to
            // get offsets to the epoch that were consistent to within
            // around 1ms.
            do {
                post = std::chrono::system_clock::now();
            } while (pre == post);

            // Now read the high-resolution timer to find out the time
            // equivalent to the post time on the system clock.
            std::chrono::high_resolution_clock::time_point high =
                std::chrono::high_resolution_clock::now();

            // Now convert both the hi-resolution clock time and the
            // post-tick system clock time into struct timevals and
            // store the difference between them as the offset.
            std::time_t high_secs =
                std::chrono::duration_cast<std::chrono::seconds>(
                    high.time_since_epoch())
                    .count();
            std::chrono::high_resolution_clock::time_point
                fractional_high_secs = high - std::chrono::seconds(high_secs);
            struct timeval high_time;
            high_time.tv_sec = static_cast<unsigned long>(high_secs);
            high_time.tv_usec = static_cast<unsigned long>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    fractional_high_secs.time_since_epoch())
                    .count());

            std::time_t post_secs =
                std::chrono::duration_cast<std::chrono::seconds>(
                    post.time_since_epoch())
                    .count();
            std::chrono::system_clock::time_point fractional_post_secs =
                post - std::chrono::seconds(post_secs);
            struct timeval post_time;
            post_time.tv_sec = static_cast<unsigned long>(post_secs);
            post_time.tv_usec = static_cast<unsigned long>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    fractional_post_secs.time_since_epoch())
                    .count());

            hr_offset = vrpn_TimevalDiff(post_time, high_time);

            // We've found our offset ... re-use it from here on.
            hr_offset_determined = true;
        }
        hr_offset_semaphore.unlock();
    }

    // The offset has been determined, by us or someone else.  Apply it.
    return vrpn_TimevalSum(hi_res_time, hr_offset);
}

int vrpn_gettimeofday(timeval *tp, void *tzp)
{
    // If we have nothing to fill in, don't try.
    if (tp == NULL) {
        return 0;
    }
	struct timezone *timeZone = reinterpret_cast<struct timezone *>(tzp);

    // Find out the time, and how long it has been in seconds since the
    // epoch.
    std::chrono::high_resolution_clock::time_point now =
        std::chrono::high_resolution_clock::now();
    std::time_t secs =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
            .count();

    // Subtract the time in seconds from the full time to get a
    // remainder that is a fraction of a second since the epoch.
    std::chrono::high_resolution_clock::time_point fractional_secs =
        now - std::chrono::seconds(secs);

    // Store the seconds and the fractional seconds as microseconds into
    // the timeval structure.  Then convert from the hi-res clock time
    // to system clock time.
    struct timeval hi_res_time;
    hi_res_time.tv_sec = static_cast<unsigned long>(secs);
    hi_res_time.tv_usec = static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            fractional_secs.time_since_epoch())
            .count());
    *tp = high_resolution_time_to_system_time(hi_res_time);

    // @todo Fill in timezone structure with relevant info.
    if (timeZone != NULL) {
		timeZone->tz_minuteswest = 0;
		timeZone->tz_dsttime = 0;
    }

    return 0;
}

// End of the section dealing with vrpn_gettimeofday()
//=====================================================================

bool vrpn_test_pack_unpack(void)
{
    // Get a buffer to use that is large enough to test all of the routines.
    vrpn_float64 dbuffer[256];
    vrpn_int32 buflen;

    vrpn_float64 in_float64 = 42.1;
    vrpn_int32 in_int32 = 17;
    vrpn_uint16 in_uint16 = 397;
    vrpn_uint8 in_uint8 = 1;

    vrpn_float64 out_float64;
    vrpn_int32 out_int32;
    vrpn_uint16 out_uint16;
    vrpn_uint8 out_uint8;

    // Test packing using little-endian routines.
    // IMPORTANT: Do these from large to small to get good alignment.
    char *bufptr = (char *)dbuffer;
    buflen = sizeof(dbuffer);
    if (vrpn_buffer_to_little_endian(&bufptr, &buflen, in_float64) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer little endian\n");
        return false;
    }
    if (vrpn_buffer_to_little_endian(&bufptr, &buflen, in_int32) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer little endian\n");
        return false;
    }
    if (vrpn_buffer_to_little_endian(&bufptr, &buflen, in_uint16) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer little endian\n");
        return false;
    }
    if (vrpn_buffer_to_little_endian(&bufptr, &buflen, in_uint8) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer little endian\n");
        return false;
    }

    // Test unpacking using little-endian routines.
    bufptr = (char *)dbuffer;
    if (in_float64 !=
        (out_float64 =
             vrpn_unbuffer_from_little_endian<vrpn_float64>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer little endian\n");
        return false;
    }
    if (in_int32 !=
        (out_int32 = vrpn_unbuffer_from_little_endian<vrpn_int32>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer little endian\n");
        return false;
    }
    if (in_uint16 !=
        (out_uint16 = vrpn_unbuffer_from_little_endian<vrpn_uint16>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer little endian\n");
        return false;
    }
    if (in_uint8 !=
        (out_uint8 = vrpn_unbuffer_from_little_endian<vrpn_uint8>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer little endian\n");
        return false;
    }

    // Test packing using big-endian routines.
    bufptr = (char *)dbuffer;
    buflen = sizeof(dbuffer);
    if (vrpn_buffer(&bufptr, &buflen, in_float64) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer big endian\n");
        return false;
    }
    if (vrpn_buffer(&bufptr, &buflen, in_int32) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer big endian\n");
        return false;
    }
    if (vrpn_buffer(&bufptr, &buflen, in_uint16) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer big endian\n");
        return false;
    }
    if (vrpn_buffer(&bufptr, &buflen, in_uint8) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer big endian\n");
        return false;
    }

    // Test unpacking using big-endian routines.
    bufptr = (char *)dbuffer;
    if (in_float64 != (out_float64 = vrpn_unbuffer<vrpn_float64>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer big endian\n");
        return false;
    }
    if (in_int32 != (out_int32 = vrpn_unbuffer<vrpn_int32>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer big endian\n");
        return false;
    }
    if (in_uint16 != (out_uint16 = vrpn_unbuffer<vrpn_uint16>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer big endian\n");
        return false;
    }
    if (in_uint8 != (out_uint8 = vrpn_unbuffer<vrpn_uint8>(bufptr))) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not unbuffer big endian\n");
        return false;
    }

    // XXX Test pack/unpack of all other types.

    // Test packing little-endian and unpacking big-endian; they should
    // be different.
    bufptr = (char *)dbuffer;
    buflen = sizeof(dbuffer);
    if (vrpn_buffer_to_little_endian(&bufptr, &buflen, in_float64) != 0) {
        fprintf(stderr,
                "vrpn_test_pack_unpack(): Could not buffer little endian\n");
        return false;
    }
    bufptr = (char *)dbuffer;
    if (in_float64 == (out_float64 = vrpn_unbuffer<vrpn_float64>(bufptr))) {
        fprintf(
            stderr,
            "vrpn_test_pack_unpack(): Cross-packing produced same result\n");
        return false;
    }

    return true;
}

bool vrpn_test_vrpn_vector(void)
{
  // Test the default constructor and ensure that the destructor doesn't crash.
  {
    vrpn_vector<int> v0;
    if ((v0.size() != 0) || (v0.data() != 0)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Default constructor failed\n");
      return false;
    }
  }
  
  // Test the sized contructor, writing to the vector, the copy
  // constructor, and the [] operator.
  {
    vrpn_uint32 count = 19;
    vrpn_vector<vrpn_uint32> v1(count);
    if (v1.size() != count) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Sized constructor size failed\n");
      return false;
    }
    for (vrpn_uint32 i = 0; i < v1.size(); i++) {
      v1[i] = i;
    }
    vrpn_vector<vrpn_uint32> v2(v1);
    if (v2.size() != count) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Copy constructor size failed\n");
      return false;
    }
    for (vrpn_uint32 i = 0; i < v2.size(); i++) {
      if (v1[i] != i) {
        fprintf(stderr,"vrpn_test_vrpn_vector(): Copy constructor data failed\n");
        return false;
      }
    }
  }
  
  // Test range constructor
  {
    vrpn_uint32 count = 19;
    vrpn_vector<vrpn_uint32> v1(count);
    for (vrpn_uint32 i = 0; i < v1.size(); i++) {
      v1[i] = i;
    }
    vrpn_vector<vrpn_uint32> v2(&v1.data()[1], &v1.data()[10]);
    if (v2.size() != 9) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Range constructor size failed\n");
      return false;
    }
    for (vrpn_uint32 i = 0; i < v2.size(); i++) {
      if (v2[i] != i+1) {
        fprintf(stderr,"vrpn_test_vrpn_vector(): Range constructor data failed\n");
        return false;
      }
    }
  }
  
  // Test data() push_back() and empty().
  {
    vrpn_vector<vrpn_uint32> v1;
    if (!v1.empty()) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Empty failed when empty\n");
      return false;
    }
    v1.push_back(1);
    if (v1.empty()) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): Empty failed when not empty\n");
      return false;
    }
    if (v1.data()[0] != 1) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): data() failed on push_back\n");
      return false;
    }
  }
  
  // Test resize().
  {
    vrpn_vector<vrpn_uint32> v1;
    v1.push_back(1);
    v1.push_back(2);
    v1.resize(2);
    if (v1.size() != 2) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize failed on do nothing\n");
      return false;
    }
    if ((v1[0] != 1) || (v1[1] != 2)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize data failed on do nothing\n");
      return false;
    }
    v1.resize(1);
    if (v1.size() != 1) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize failed on shrink\n");
      return false;
    }
    if ((v1[0] != 1)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize data failed on shrink\n");
      return false;
    }
    v1.resize(10);
    if (v1.size() != 10) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize failed on grow\n");
      return false;
    }
    if ((v1[0] != 1)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): resize data failed on grow\n");
      return false;
    }
  }
  
  // Test front(), back(), assign(), and clear()
  {
    vrpn_vector<vrpn_uint32> v1;
    v1.push_back(1);
    v1.push_back(2);
    if (v1.front() != 1) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): front failed\n");
      return false;
    }
    if (v1.back() != 2) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): back failed\n");
      return false;
    }
    vrpn_vector<vrpn_uint32> v2;
    vrpn_uint32 count = 2;
    v2.assign(count, 7);
    if ((v2[0] != 7) || (v2[1] != 7)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): assign value failed\n");
      return false;
    }
    v2.assign(&v1[0], &v1[2]);
    if ((v2[0] != 1) || (v2[1] != 2)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): assign iterators failed\n");
      return false;
    }
    v1.clear();
    if (v1.size() != 0) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): clear failed\n");
      return false;
    }
  }
  
  // Test begin() and end()
  {
    vrpn_vector<vrpn_uint32> v0, v1(10);
    if ((v0.begin() != 0) || (v0.end() != 0)) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): empty begin/end failed\n");
      return false;
    }
    if ((v1.begin() != v1.data()) || (v1.end() != &v1.data()[10])) {
      fprintf(stderr,"vrpn_test_vrpn_vector(): begin/end failed\n");
      return false;
    }
  }

  // Test operator =
  {
    vrpn_vector<vrpn_uint32> v0, v1;
    v0.push_back(1);
    v0.push_back(2);
    v1 = v0;
    v1[0] = 3;
    if ((v1[1] != 2) || (v0[0] != 1)) {
      fprintf(stderr, "vrpn_test_vrpn_vector(): operator = failed (%d, %d)\n", v0[0], v1[1]);
      return false;
    }
  }

  return true;
}
