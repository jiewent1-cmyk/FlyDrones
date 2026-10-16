/* ECPS295: Linux i2c-dev platform functions + a small C ABI (ctypes) for the VL53L7CX ULD.
   I2C transfers: 16-bit register index, big-endian; writes and reads split into CHUNK-byte transfers with the index
   advanced per chunk (the 84 kB firmware upload goes through WrMulti). */
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include "vl53l7cx_class.h"

#ifndef CHUNK
#define CHUNK 1024
#endif

static int xfer(int fd, struct i2c_msg *m, int n) {
  struct i2c_rdwr_ioctl_data d = {m, (unsigned)n};
  for (int tries = 0; tries < 3; ++tries)
    if (ioctl(fd, I2C_RDWR, &d) == n) return 0;
  return 1;
}

uint8_t VL53L7CX::WrMulti(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t size) {
  static uint8_t buf[CHUNK + 2];
  for (uint32_t i = 0; i < size;) {
    uint32_t n = size - i > CHUNK ? CHUNK : size - i;
    uint16_t r = (uint16_t)(reg + i);
    buf[0] = r >> 8; buf[1] = r & 0xFF;
    memcpy(buf + 2, v + i, n);
    struct i2c_msg m = {(uint16_t)(p->address >> 1), 0, (uint16_t)(n + 2), buf};
    if (xfer(p->fd, &m, 1)) return 1;
    i += n;
  }
  return 0;
}

uint8_t VL53L7CX::RdMulti(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t size) {
  for (uint32_t i = 0; i < size;) {
    uint32_t n = size - i > CHUNK ? CHUNK : size - i;
    uint16_t r = (uint16_t)(reg + i);
    uint8_t idx[2] = {(uint8_t)(r >> 8), (uint8_t)(r & 0xFF)};
    struct i2c_msg m[2] = {{(uint16_t)(p->address >> 1), 0, 2, idx},
                           {(uint16_t)(p->address >> 1), I2C_M_RD, (uint16_t)n, v + i}};
    if (xfer(p->fd, m, 2)) return 1;
    i += n;
  }
  return 0;
}

uint8_t VL53L7CX::RdByte(VL53L7CX_Platform *p, uint16_t reg, uint8_t *v) { return RdMulti(p, reg, v, 1); }
uint8_t VL53L7CX::WrByte(VL53L7CX_Platform *p, uint16_t reg, uint8_t v) { return WrMulti(p, reg, &v, 1); }

void VL53L7CX::SwapBuffer(uint8_t *buffer, uint16_t size) {
  for (uint32_t i = 0; i < size; i += 4) {
    uint32_t t = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | buffer[i + 3];
    memcpy(&buffer[i], &t, 4);
  }
}

uint8_t VL53L7CX::WaitMs(VL53L7CX_Platform *, uint32_t ms) {
  struct timespec ts = {(time_t)(ms / 1000), (long)(ms % 1000) * 1000000L};
  nanosleep(&ts, NULL);
  return 0;
}

/* ------------------------------------------------------------------ C ABI */
struct vl7 {
  VL53L7CX *s;
  int fd;
  VL53L7CX_ResultsData r;
};

extern "C" {
/* open /dev/i2c-<bus>, check the sensor answers at addr7; returns NULL on failure (errno-style code in *err) */
void *vl7_open(int bus, int addr7, int *err) {
  char path[32];
  snprintf(path, sizeof path, "/dev/i2c-%d", bus);
  int fd = open(path, O_RDWR);
  if (fd < 0) { *err = -errno; return NULL; }
  vl7 *h = (vl7 *)calloc(1, sizeof(vl7));
  h->fd = fd;
  h->s = new VL53L7CX(fd, (uint16_t)(addr7 << 1));
  uint8_t alive = 0;
  if (h->s->vl53l7cx_is_alive(&alive) || !alive) { *err = 1; delete h->s; close(fd); free(h); return NULL; }
  *err = 0;
  return h;
}
/* firmware upload + configuration; returns ULD status (0 ok) */
int vl7_init(void *p, int resolution, int hz, int integration_ms) {
  vl7 *h = (vl7 *)p;
  int st = h->s->vl53l7cx_init();
  if (st) return 100 + st;
  st |= h->s->vl53l7cx_set_resolution((uint8_t)resolution);
  st |= h->s->vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
  st |= h->s->vl53l7cx_set_ranging_frequency_hz((uint8_t)hz);
  if (integration_ms > 0) st |= h->s->vl53l7cx_set_integration_time_ms((uint32_t)integration_ms);
  return st;
}
int vl7_start(void *p) { return ((vl7 *)p)->s->vl53l7cx_start_ranging(); }
int vl7_stop(void *p) { return ((vl7 *)p)->s->vl53l7cx_stop_ranging(); }
int vl7_ready(void *p) {
  uint8_t r = 0;
  if (((vl7 *)p)->s->vl53l7cx_check_data_ready(&r)) return -1;
  return r;
}
/* copy one frame: dist_mm[n], status[n] (first target per zone), returns ULD status */
int vl7_read(void *p, int16_t *dist_mm, uint8_t *status, int n, int8_t *temp_c) {
  vl7 *h = (vl7 *)p;
  int st = h->s->vl53l7cx_get_ranging_data(&h->r);
  if (st) return st;
  for (int i = 0; i < n && i < 64; ++i) {
    dist_mm[i] = h->r.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE * i];
    status[i] = h->r.target_status[VL53L7CX_NB_TARGET_PER_ZONE * i];
  }
  if (temp_c) *temp_c = h->r.silicon_temp_degc;
  return 0;
}
void vl7_close(void *p) {
  vl7 *h = (vl7 *)p;
  if (!h) return;
  delete h->s;
  close(h->fd);
  free(h);
}
}
