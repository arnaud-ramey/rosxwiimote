// C
#include <stdio.h>
#include <poll.h>
extern "C" {
#include "xwiimote.h"
}
// C++
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

static struct xwii_iface *iface;
ros::Publisher joy_pub;
sensor_msgs::Joy joy_msg;
// 4 axes:
//    left-right rocker (-1 0 1)
//    up-down rocker (-1 0 1)
//    nunchuk left-right joystick (-1 .. 1)
//    nunchuk up-down joystick (-1 .. 1)
// 9 buttons:
// 	XWII_KEY_A,
//  XWII_KEY_B,
//  XWII_KEY_PLUS,
//  XWII_KEY_MINUS,
//  XWII_KEY_HOME,
//  XWII_KEY_ONE,
//  XWII_KEY_TWO,
//  XWII_KEY_C,
//  XWII_KEY_Z,


static bool run_iface(struct xwii_iface *iface) {
  struct xwii_event event;
  int ret = 0, fds_num;
  struct pollfd fds[2];

  memset(fds, 0, sizeof(fds));
  fds[0].fd = 0;
  fds[0].events = POLLIN;
  fds[1].fd = xwii_iface_get_fd(iface);
  fds[1].events = POLLIN;
  fds_num = 2;

  if (xwii_iface_watch(iface, true)) {
    ROS_FATAL("Error: Cannot initialize hotplug watch descriptor");
    return false;
  }

  unsigned int code;
  float x, y;
  bool pressed, need_pub = false;
  while (ros::ok()) {
    ret = poll(fds, fds_num, -1);
    if (ret < 0) {
      if (errno != EINTR) {
        ret = -errno;
        ROS_FATAL("Error: Cannot poll fds: %d", ret);
        break;
      }
    }

    ret = xwii_iface_dispatch(iface, &event, sizeof(event));
    if (ret) {
      if (ret != -EAGAIN)
        ROS_ERROR("Error: Read failed with err:%d", ret);
      continue;
    }
    switch (event.type) {
      case XWII_EVENT_GONE:
        ROS_WARN("Info: Device gone");
        fds[1].fd = -1;
        fds[1].events = 0;
        fds_num = 1;
        break;
      case XWII_EVENT_NUNCHUK_KEY:
      case XWII_EVENT_KEY:
        need_pub = true;
        code = event.v.key.code;
        pressed = event.v.key.state;
        //        if (pressed)
        //          ROS_INFO("Key %i pressed", code);
        //        else
        //          ROS_INFO("Key %i released", code);
        switch (code) {
          case XWII_KEY_LEFT:
            joy_msg.axes[0] = -pressed;
            break;
          case XWII_KEY_RIGHT:
            joy_msg.axes[0] =  pressed;
            break;
          case XWII_KEY_DOWN:
            joy_msg.axes[1] = -pressed;
            break;
          case XWII_KEY_UP:
            joy_msg.axes[1] =  pressed;
            break;
          // buttons
          case XWII_KEY_A:
            joy_msg.buttons[0] = pressed;
            break;
          case XWII_KEY_B:
            joy_msg.buttons[1] = pressed;
            break;
          case XWII_KEY_PLUS:
            joy_msg.buttons[2] = pressed;
            break;
          case XWII_KEY_MINUS:
            joy_msg.buttons[3] = pressed;
            break;
          case XWII_KEY_HOME:
            joy_msg.buttons[4] = pressed;
            break;
          case XWII_KEY_ONE:
            joy_msg.buttons[5] = pressed;
            break;
          case XWII_KEY_TWO:
            joy_msg.buttons[6] = pressed;
            break;
          case XWII_KEY_C:
            joy_msg.buttons[7] = pressed;
            break;
          case XWII_KEY_Z:
            joy_msg.buttons[8] = pressed;
            break;
          default:
            need_pub = false;
            break;
        }
        break;
      case XWII_EVENT_NUNCHUK_MOVE:
        //ROS_INFO_THROTTLE(1, "XWII_EVENT_NUNCHUK_MOVE:%i, %i", event.v.abs[0].x, event.v.abs[0].y);
        x = 0.01 * event.v.abs[0].x;
        if (x < -1)
          x = -1;
        else if (x > 1)
          x = 1;
        joy_msg.axes[2] = x;
        y = 0.01 * event.v.abs[0].y;
        if (y < -1)
          y = -1;
        else if (y > 1)
          y = 1;
        joy_msg.axes[3] = y;
        need_pub = true;
        break;
#if 0
      case XWII_EVENT_WATCH:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_WATCH");
        break;
      case XWII_EVENT_ACCEL:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_ACCEL");
        break;
      case XWII_EVENT_IR:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_IR");
        break;
      case XWII_EVENT_MOTION_PLUS:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_MOTION_PLUS");
        break;
      case XWII_EVENT_CLASSIC_CONTROLLER_KEY:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_KEY");
        break;
      case XWII_EVENT_CLASSIC_CONTROLLER_MOVE:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_MOVE");
        break;
      case XWII_EVENT_BALANCE_BOARD:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_BALANCE_BOARD");
        break;
      case XWII_EVENT_PRO_CONTROLLER_KEY:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_PRO_CONTROLLER_KEY");
        break;
      case XWII_EVENT_PRO_CONTROLLER_MOVE:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_WAXWII_EVENT_PRO_CONTROLLER_MOVETCH");
        break;
      case XWII_EVENT_GUITAR_KEY:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_KEY");
        break;
      case XWII_EVENT_GUITAR_MOVE:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_MOVE");
        break;
      case XWII_EVENT_DRUMS_KEY:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_KEY");
        break;
      case XWII_EVENT_DRUMS_MOVE:
        ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_MOVE");
        break;
#endif
      default:
        break;
    } // end switch
    if (need_pub) {
      joy_msg.header.stamp = ros::Time::now();
      joy_pub.publish(joy_msg);
      ros::spinOnce();
    }
  } // end while ros::ok
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

static char *get_dev(int num) {
  struct xwii_monitor *mon = xwii_monitor_new(false, false);
  if (!mon) {
    ROS_WARN("Cannot create monitor");
    return NULL;
  }

  char *ent;
  int i = 0;
  while ((ent = xwii_monitor_poll(mon))) {
    if (++i == num)
      break;
    free(ent);
  }

  xwii_monitor_unref(mon);
  if (!ent)
    ROS_WARN("Cannot find device with number #%d", num);

  return ent;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "xwiimote");
  std::string device_path = "";
  int device_idx = -1;
  ros::NodeHandle nh_public, nh_private("~");
  // params
  nh_private.param("device_idx", device_idx, device_idx);
  nh_private.param("device_path", device_path, device_path);
  // pubs
  joy_msg.axes.resize(4);
  joy_msg.buttons.resize(9);
  joy_pub = nh_private.advertise<sensor_msgs::Joy>("joy", 1);

  if (device_idx > 0) {
    device_path = get_dev(device_idx);
    if (device_path.empty()) {
      ROS_FATAL("Cannot find device %i", device_idx);
      return -1;
    }
  }
  else if (device_path.empty()) {
    ROS_FATAL("You need to specify either device_idx or device_path!");
    return -1;
  }

  int ret = xwii_iface_new(&iface, device_path.c_str());
  if (ret) {
    ROS_FATAL("Cannot create xwii_iface '%s' err:%d", device_path.c_str(), ret);
    return -1;
  }

  ret = xwii_iface_open(iface, xwii_iface_available(iface) | XWII_IFACE_WRITABLE);
  if (ret) {
    ROS_FATAL("Error: Cannot open interface: %d", ret);
    return -1;
  }
  ROS_INFO("Successfully open Wiimote device '%s'", device_path.c_str());

  ret = run_iface(iface);
  if (ret) {
    ROS_FATAL("Program failed; press any key to exit");
    return -1;
  }
  xwii_iface_unref(iface);
}
