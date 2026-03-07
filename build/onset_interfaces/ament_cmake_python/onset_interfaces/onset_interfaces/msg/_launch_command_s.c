// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "onset_interfaces/msg/detail/launch_command__struct.h"
#include "onset_interfaces/msg/detail/launch_command__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool onset_interfaces__msg__launch_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("onset_interfaces.msg._launch_command.LaunchCommand", full_classname_dest, 50) == 0);
  }
  onset_interfaces__msg__LaunchCommand * ros_message = _ros_message;
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_launch
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_launch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_launch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_turret
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_turret");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_turret = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // home_onset_request
    PyObject * field = PyObject_GetAttrString(_pymsg, "home_onset_request");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->home_onset_request = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * onset_interfaces__msg__launch_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LaunchCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("onset_interfaces.msg._launch_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LaunchCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  onset_interfaces__msg__LaunchCommand * ros_message = (onset_interfaces__msg__LaunchCommand *)raw_ros_message;
  {  // velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_launch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_launch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_launch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_turret
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_turret);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_turret", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // home_onset_request
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->home_onset_request);
    {
      int rc = PyObject_SetAttrString(_pymessage, "home_onset_request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
