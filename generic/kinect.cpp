#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/kinect.cpp"
#else

#include <pthread.h>
#include <assert.h>

#ifndef _FREENECT_WRAPPER_
#define _FREENECT_WRAPPER_
static freenect_context *f_ctx;
static freenect_device *f_dev;
static pthread_t freenect_thread;
volatile int die = 0;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint16_t *depth_mid, *depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;

int freenect_angle = 0;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;

uint16_t t_gamma[2048];

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
  uint16_t *depth = (uint16_t*)v_depth;

  pthread_mutex_lock(&gl_backbuf_mutex);
  for (int i = 0; i < 640*480; i++) {
    depth_mid[i] = depth[i];

//     int pval = t_gamma[depth[i]];
//     int lb = pval & 0xff;
//     switch (pval>>8) {
//     case 0:
//       depth_mid[3*i+0] = 255;
//       depth_mid[3*i+1] = 255-lb;
//       depth_mid[3*i+2] = 255-lb;
//       break;
//     case 1:
//       depth_mid[3*i+0] = 255;
//       depth_mid[3*i+1] = lb;
//       depth_mid[3*i+2] = 0;
//       break;
//     case 2:
//       depth_mid[3*i+0] = 255-lb;
//       depth_mid[3*i+1] = 255;
//       depth_mid[3*i+2] = 0;
//       break;
//     case 3:
//       depth_mid[3*i+0] = 0;
//       depth_mid[3*i+1] = 255;
//       depth_mid[3*i+2] = lb;
//       break;
//     case 4:
//       depth_mid[3*i+0] = 0;
//       depth_mid[3*i+1] = 255-lb;
//       depth_mid[3*i+2] = 255;
//       break;
//     case 5:
//       depth_mid[3*i+0] = 0;
//       depth_mid[3*i+1] = 0;
//       depth_mid[3*i+2] = 255-lb;
//       break;
//     default:
//       depth_mid[3*i+0] = 0;
//       depth_mid[3*i+1] = 0;
//       depth_mid[3*i+2] = 0;
//       break;
//     }
  }
  got_depth++;
  pthread_cond_signal(&gl_frame_cond);
  pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
  pthread_mutex_lock(&gl_backbuf_mutex);

  // swap buffers
  assert (rgb_back == rgb);
  rgb_back = rgb_mid;
  freenect_set_video_buffer(dev, rgb_back);
  rgb_mid = (uint8_t*)rgb;

  got_rgb++;
  pthread_cond_signal(&gl_frame_cond);
  pthread_mutex_unlock(&gl_backbuf_mutex);
}

static void *freenect_threadfunc(void *arg) {
  int accelCount = 0;

  freenect_set_tilt_degs(f_dev,freenect_angle);
  freenect_set_led(f_dev,LED_RED);
  freenect_set_depth_callback(f_dev, depth_cb);
  freenect_set_video_callback(f_dev, rgb_cb);
  freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
  freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_set_video_buffer(f_dev, rgb_back);

  freenect_start_depth(f_dev);
  freenect_start_video(f_dev);

  printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");

  while (!die && freenect_process_events(f_ctx) >= 0) {
    //Throttle the text output
    if (accelCount++ >= 2000)
      {
        accelCount = 0;
        freenect_raw_tilt_state* state;
        freenect_update_tilt_state(f_dev);
        state = freenect_get_tilt_state(f_dev);
        double dx,dy,dz;
        freenect_get_mks_accel(state, &dx, &dy, &dz);
        printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
        fflush(stdout);
      }

    if (requested_format != current_format) {
      freenect_stop_video(f_dev);
      freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
      freenect_start_video(f_dev);
      current_format = requested_format;
    }
  }

  printf("\nshutting down streams...\n");

  freenect_stop_depth(f_dev);
  freenect_stop_video(f_dev);

  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);

  printf("-- done!\n");
  return NULL;
}
#endif

static int kinect_(init)(lua_State *L) {

  int user_device_number = 0;
  if (lua_isnumber(L, 1)) {
    user_device_number = lua_tonumber(L, 1);
  }

  depth_mid = (uint16_t*) malloc(640*480*2);
  depth_front = (uint16_t*) malloc(640*480*2);
  rgb_back = (uint8_t*) malloc(640*480*3);
  rgb_mid = (uint8_t*) malloc(640*480*3);
  rgb_front = (uint8_t*) malloc(640*480*3);

  if (freenect_init(&f_ctx, NULL) < 0) {
    printf("freenect_init() failed\n");
    return 0;
  }

  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  int nr_devices = freenect_num_devices (f_ctx);
  printf ("<libkinect> number of devices found: %d\n", nr_devices);

  if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
    printf("could not open device\n");
    return 0;
  }

  if (pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL)) {
    printf("pthread_create failed\n");
    return 0;
  }

  return 0;
}

static int kinect_(stop)(lua_State *L) {
}

static int kinect_(getRGBD)(lua_State *L) {
  THTensor *rgb = (THTensor *)luaT_checkudata(L, 1, torch_(Tensor_id));
  THTensor *depth = (THTensor *)luaT_checkudata(L, 2, torch_(Tensor_id));

  pthread_mutex_lock(&gl_backbuf_mutex);

  // When using YUV_RGB mode, RGB frames only arrive at 15Hz, so we shouldn't force them to draw in lock-step.
  // However, this is CPU/GPU intensive when we are receiving frames in lockstep.
  if (current_format == FREENECT_VIDEO_YUV_RGB) {
    while (!got_depth && !got_rgb) {
      pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
    }
  } else {
    while ((!got_depth || !got_rgb) && requested_format != current_format) {
      pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
    }
  }

  if (requested_format != current_format) {
    pthread_mutex_unlock(&gl_backbuf_mutex);
    return 0;
  }

  uint8_t *tmp;
  uint16_t *tmpd;
  if (got_depth) {
    tmpd = depth_front;
    depth_front = depth_mid;
    depth_mid = tmpd;
    got_depth = 0;
  }
  if (got_rgb) {
    tmp = rgb_front;
    rgb_front = rgb_mid;
    rgb_mid = tmp;
    got_rgb = 0;
  }

  pthread_mutex_unlock(&gl_backbuf_mutex);

  if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB) {
    // 4 channels
    THTensor_(resize3d)(rgb, 3, 480, 640);
    THTensor_(resize3d)(depth, 1, 480, 640);

    // copy
    real *rgb_data = THTensor_(data)(rgb);
    for (int c = 0; c < 3; c++) {
      for (int k = 0; k < 640*480; k++) {
        rgb_data[c*640*480 + k] = (real)rgb_front[k*3 + c];
      }
    }

  } else {
    // 2 channel
    THTensor_(resize3d)(rgb, 1, 480, 640);
    THTensor_(resize3d)(depth, 1, 480, 640);

    // copy
    real *rgb_data = THTensor_(data)(rgb);
    for (int k = 0; k < 640*480*1; k++) {
      rgb_data[k] = (real)rgb_front[k];
    }
  }

  // copy depth
  real *depth_data = THTensor_(data)(depth);
  for (int k = 0; k < 640*480; k++) {
      depth_data[k] = (real)depth_front[k];
  }

  return 0;
}

extern "C" {
  static const struct luaL_Reg kinect_(methods__) [] = {
    {"init", kinect_(init)},
    {"getRGBD", kinect_(getRGBD)},
    {NULL, NULL}
  };

  static void kinect_(Init)(lua_State *L)
  {
    luaT_pushmetaclass(L, torch_(Tensor_id));
    luaT_registeratname(L, kinect_(methods__), "kinect");
    lua_pop(L,1);
  }
}

#endif
