// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from onset_interfaces:msg/STM32State.idl
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
#include "onset_interfaces/msg/detail/stm32_state__struct.h"
#include "onset_interfaces/msg/detail/stm32_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool onset_interfaces__msg__stm32_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("onset_interfaces.msg._stm32_state.STM32State", full_classname_dest, 44) == 0);
  }
  onset_interfaces__msg__STM32State * ros_message = _ros_message;
  {  // sw1
    PyObject * field = PyObject_GetAttrString(_pymsg, "sw1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sw1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sw2
    PyObject * field = PyObject_GetAttrString(_pymsg, "sw2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sw2 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sw3
    PyObject * field = PyObject_GetAttrString(_pymsg, "sw3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sw3 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_code = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * onset_interfaces__msg__stm32_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of STM32State */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("onset_interfaces.msg._stm32_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "STM32State");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  onset_interfaces__msg__STM32State * ros_message = (onset_interfaces__msg__STM32State *)raw_ros_message;
  {  // sw1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sw1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sw1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sw2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sw2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sw2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sw3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sw3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sw3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
