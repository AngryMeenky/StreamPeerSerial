/*************************************************************************/
/*  serial_port_config.h                                                 */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2023 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2023 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef SERIAL_PORT_CONFIG_H
#define SERIAL_PORT_CONFIG_H

#ifdef GDEXTENSION
#include <godot_cpp/variant/builtin_types.hpp>
#include <godot_cpp/classes/ref.hpp>

using namespace godot;
#else
#include "core/object/ref_counted.h"
#endif

#include "libserialport/libserialport.h"

class SerialPortConfig : public RefCounted {
  GDCLASS(SerialPortConfig, RefCounted);

protected:
  sp_port_config *_config = nullptr;

  static void _bind_methods();

public:
  enum Parity {
    INVALID_PARITY = SP_PARITY_INVALID,
    NO_PARITY      = SP_PARITY_NONE,
    ODD_PARITY     = SP_PARITY_ODD,
    EVEN_PARITY    = SP_PARITY_EVEN,
    MARK_PARITY    = SP_PARITY_MARK,
    SPACE_PARITY   = SP_PARITY_SPACE,
  };

  enum Rts {
    INVALID_RTS = SP_RTS_INVALID,
    RTS_OFF     = SP_RTS_OFF,
    RTS_ON      = SP_RTS_ON,
    RTS_FLOW    = SP_RTS_FLOW_CONTROL,
  };

  enum Cts {
    INVALID_CTS = SP_CTS_INVALID,
    CTS_IGNORE  = SP_CTS_IGNORE,
    CTS_FLOW    = SP_CTS_FLOW_CONTROL,
  };

  enum Dtr {
    INVALID_DTR = SP_DTR_INVALID,
    DTR_OFF     = SP_DTR_OFF,
    DTR_ON      = SP_DTR_ON,
    DTR_FLOW    = SP_DTR_FLOW_CONTROL,
  };

  enum Dsr {
    INVALID_DSR = SP_DSR_INVALID,
    DSR_IGNORE  = SP_DSR_IGNORE,
    DSR_FLOW    = SP_DSR_FLOW_CONTROL,
  };

  enum XonXoff {
    INVALID_XONXOFF  = SP_XONXOFF_INVALID,
    XONXOFF_DISABLED = SP_XONXOFF_DISABLED,
    XONXOFF_IN       = SP_XONXOFF_IN,
    XONXOFF_OUT      = SP_XONXOFF_OUT,
    XONXOFF_INOUT    = SP_XONXOFF_INOUT,
  };

  enum FlowControl {
    FLOW_NONE    = SP_FLOWCONTROL_NONE,
    FLOW_XONXOFF = SP_FLOWCONTROL_XONXOFF,
    FLOW_RTSCTS  = SP_FLOWCONTROL_RTSCTS,
    FLOW_DTRDSR  = SP_FLOWCONTROL_DTRDSR,
  };

  SerialPortConfig();
  ~SerialPortConfig();

        sp_port_config *ptr()       { return _config; }
  const sp_port_config *ptr() const { return _config; }

  sp_port_config *swap(sp_port_config *config);
  void            replace(sp_port_config *config);

  bool    has_baudrate() const;
  int64_t get_baudrate() const;
  Error   set_baudrate(int64_t rate);

  bool    has_data_bits() const;
  int64_t get_data_bits() const;
  Error   set_data_bits(int64_t bits);

  bool    has_stop_bits() const;
  int64_t get_stop_bits() const;
  Error   set_stop_bits(int64_t bits);

  bool   has_parity() const;
  Parity get_parity() const;
  Error  set_parity(Parity parity);

  bool  has_rts() const;
  Rts   get_rts() const;
  Error set_rts(Rts rts);

  bool  has_cts() const;
  Cts   get_cts() const;
  Error set_cts(Cts cts);

  bool  has_dtr() const;
  Dtr   get_dtr() const;
  Error set_dtr(Dtr dts);

  bool  has_dsr() const;
  Dsr   get_dsr() const;
  Error set_dsr(Dsr dsr);

  bool    has_xon_xoff() const;
  XonXoff get_xon_xoff() const;
  Error   set_xon_xoff(XonXoff xon_xoff);

  Error set_flow_control(FlowControl flow_control);
};


VARIANT_ENUM_CAST(SerialPortConfig::Parity);
VARIANT_ENUM_CAST(SerialPortConfig::Rts);
VARIANT_ENUM_CAST(SerialPortConfig::Cts);
VARIANT_ENUM_CAST(SerialPortConfig::Dtr);
VARIANT_ENUM_CAST(SerialPortConfig::Dsr);
VARIANT_ENUM_CAST(SerialPortConfig::XonXoff);
VARIANT_ENUM_CAST(SerialPortConfig::FlowControl);

#endif // SERIAL_PORT_CONFIG_H
