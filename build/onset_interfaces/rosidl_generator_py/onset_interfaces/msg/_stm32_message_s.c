// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from onset_interfaces:msg/STM32Message.idl
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
#include "onset_interfaces/msg/detail/stm32_message__struct.h"
#include "onset_interfaces/msg/detail/stm32_message__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool onset_interfaces__msg__stm32_message__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("onset_interfaces.msg._stm32_message.STM32Message", full_classname_dest, 48) == 0);
  }
  onset_interfaces__msg__STM32Message * ros_message = _ros_message;
  {  // angle_launch
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_launch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_launch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // power_on_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "power_on_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->power_on_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // home_elbow_request
    PyObject * field = PyObject_GetAttrString(_pymsg, "home_elbow_request");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->home_elbow_request = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * onset_interfaces__msg__stm32_message__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of STM32Message */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("onset_interfaces.msg._stm32_message");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "STM32Message");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  onset_interfaces__msg__STM32Message * ros_message = (onset_interfaces__msg__STM32Message *)raw_ros_message;
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
  {  // power_on_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->power_on_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "power_on_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // home_elbow_request
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->home_elbow_request);
    {
      int rc = PyObject_SetAttrString(_pymessage, "home_elbow_request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
