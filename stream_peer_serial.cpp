/*************************************************************************/
/*  serial_port.cpp                                                      */
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

#include "stream_peer_serial.h"

#ifdef GDEXTENSION
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;
#else
#include "core/object/class_db.h"
#include "core/os/memory.h"
#include "core/os/os.h"
#endif
#include <string>

using namespace std::chrono;


void StreamPeerSerial::_data_received(const PackedByteArray &buf) {
	emit_signal("data_received", buf);
}


StreamPeerSerial::StreamPeerSerial(const String &port, uint32_t baudrate, uint32_t timeout, ByteSize bytesize, Parity parity, StopBits stopbits, FlowControl flowcontrol) {
  serialerror_t err = serialerror_success;
	serial = new Serial(port.ascii().get_data(),
			baudrate, Timeout::simpleTimeout(timeout), bytesize_t(bytesize),
      parity_t(parity), stopbits_t(stopbits), flowcontrol_t(flowcontrol), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }
}


StreamPeerSerial::~StreamPeerSerial() {
	close();
	stop_monitoring();
	delete serial;
}


Dictionary StreamPeerSerial::list_ports() {
	std::vector<PortInfo> ports_info = serial::list_ports();

	Dictionary info_dict;
	for (PortInfo port : ports_info) {
		Dictionary info;
		info["desc"] = port.description.c_str();
		info["hw_id"] = port.hardware_id.c_str();
		info_dict[port.port.c_str()] = Variant(info);
	}

	return info_dict;
}


void StreamPeerSerial::_on_error(const String &where, const String &what) {
	fine_working = false;
	error_message = "[" + get_port() + "] Error at " + where + ": " + what;
	// ERR_FAIL_MSG(error_message);
	emit_signal("got_error", where, what);
}


Error StreamPeerSerial::start_monitoring(uint64_t interval_in_usec) {
	ERR_FAIL_COND_V_MSG(!monitoring_should_exit, ERR_ALREADY_IN_USE, "Monitor already started.");
	stop_monitoring();
	monitoring_should_exit = false;
	monitoring_interval = interval_in_usec;
	thread = std::thread(_thread_func, this);
	fine_working = is_open();

	return OK;
}


void StreamPeerSerial::stop_monitoring() {
	monitoring_should_exit = true;
	if (thread.joinable()) {
		thread.join();
	}
}


void StreamPeerSerial::_thread_func(void *p_user_data) {
	StreamPeerSerial *serial_port = static_cast<StreamPeerSerial *>(p_user_data);
	while (!serial_port->monitoring_should_exit) {
		time_point time_start = system_clock::now();

		if (serial_port->fine_working) {
			if (serial_port->is_open() && serial_port->get_available_bytes() > 0) {
				serial_port->call_deferred("_data_received", serial_port->read_raw(serial_port->get_available_bytes()));
			}
		}
		time_t time_elapsed = duration_cast<microseconds>(system_clock::now() - time_start).count();
		if (time_elapsed < serial_port->monitoring_interval) {
			std::this_thread::sleep_for(microseconds(serial_port->monitoring_interval - time_elapsed));
		}
	}
}

Error StreamPeerSerial::open(String port) {
  error_message = "";
  if(serial->isOpen()) {
    close();
  }

  if(!port.is_empty()) {
    if(set_port(port) != OK) {
      _on_error(__FUNCTION__, serial->getLastError().c_str());
      return FAILED;
    }
  }

  if(auto err = serial->open(); err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    switch(err) {
      case serialerror_io_failed:
        return ERR_CANT_OPEN;
      break;
      case serialerror_serial:
        return ERR_ALREADY_IN_USE;
      break;
      case serialerror_argument:
        return ERR_INVALID_PARAMETER;
      break;
      default:
        return FAILED;
      break;
    }
  }

  fine_working = true;
  emit_signal("opened", port);
  return OK;
}


bool StreamPeerSerial::is_open() const {
	return serial->isOpen();
}

void StreamPeerSerial::close() {
  if(serial->close() != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  fine_working = false;
  emit_signal("closed", serial->getPort().c_str());
}


Error StreamPeerSerial::_put_bytes(const uint8_t *ptr, int32_t bytes) {
  if(bytes < 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  Error result = OK;
  serialerror_t err = serialerror_success;
  do {
    if(int sent = static_cast<int>(serial->write(ptr, bytes, &err)); sent == 0) {
      std::this_thread::sleep_for(milliseconds(serial->getTimeout().write_timeout_constant));
    }
    else {
      ptr += sent;
      bytes -= sent;
    }
  } while(err == serialerror_success && bytes > 0);

  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    switch(err) {
      case serialerror_not_opened:
          result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_io_failed:
        result = ERR_FILE_CANT_WRITE;
      break;
      case serialerror_serial:
        result = ERR_FILE_CORRUPT;
      break;
      default:
        result = ERR_UNAVAILABLE;
      break;
    }
  }

  return result;
}


Error StreamPeerSerial::put_data(const PackedByteArray &data) {
  return _put_bytes(data.ptr(), data.size());
}


Array StreamPeerSerial::put_partial_data(const PackedByteArray &data) {
  Array result;
  serialerror_t err = serialerror_success;

  if(auto sent = serial->write(data.ptr(), data.size(), &err); sent == 0) {
    result = Array(data);
  }
  else if(sent < data.size()) {
    result = Array(data.slice(sent));
  }

  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

	return result;
}


Error StreamPeerSerial::_get_bytes(uint8_t *ptr, int32_t bytes, int32_t &count) {
  count = 0;

  if(bytes <= 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  Error result = OK;
  serialerror_t err = serialerror_success;
  do {
    if(auto received = serial->read(ptr, bytes, &err); received == 0) {
      std::this_thread::sleep_for(milliseconds(serial->getTimeout().read_timeout_constant));
    }
    else {
      ptr += received;
      bytes -= received;
      count += received;
    }
  } while(err == serialerror_success && bytes > 0);

  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    switch(err) {
      case serialerror_not_opened:
          result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_io_failed:
        result = ERR_FILE_CANT_READ;
      break;
      case serialerror_serial:
        result = ERR_CANT_ACQUIRE_RESOURCE;
      break;
      default:
        result = FAILED;
      break;
    }
  }

  return result;
}


Array StreamPeerSerial::get_data(int32_t bytes) {
  if(bytes < 0) {
    return Array();
  }

  int32_t count;
  PackedByteArray buffer;
  buffer.resize(bytes);
  (void) _get_bytes(buffer.ptrw(), bytes, count);
  if(count != bytes) {
    buffer.resize(count);
  }

  return Array(buffer);
}


Array StreamPeerSerial::get_partial_data(int32_t bytes) {
  auto avail = get_available_bytes();
  if(avail <= 0) {
    return Array();
  }
  else if(avail < bytes) {
    bytes = avail;
  }

  return get_data(bytes);
}


int32_t StreamPeerSerial::get_available_bytes() const {
  serialerror_t err = serialerror_success;
  auto retVal = serial->available(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return retVal;
}


void StreamPeerSerial::put_8(int32_t value) {
  put_u8(value & 0xFF);
}

void StreamPeerSerial::put_u8(uint32_t value) {
  auto val = static_cast<uint8_t>(value);
  (void) _put_bytes(&val, 1);
}

void StreamPeerSerial::put_16(int32_t value) {
  put_u16(static_cast<uint32_t>(value));
}

void StreamPeerSerial::put_u16(uint32_t value) {
  uint8_t buffer[2];
  if(big_endian) {
    buffer[0] = value >> 8;
    buffer[1] = value & 0xFF;
  }
  else {
    buffer[0] = value & 0xFF;
    buffer[1] = value >> 8;
  }

  (void) _put_bytes(&buffer[0], sizeof(buffer));
}

void StreamPeerSerial::put_32(int32_t value) {
  put_u32(static_cast<uint32_t>(value));
}

void StreamPeerSerial::put_u32(uint32_t value) {
  uint8_t buffer[4];
  if(big_endian) {
    buffer[0] = (value >> 24);
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >>  8) & 0xFF;
    buffer[3] =  value        & 0xFF;
  }
  else {
    buffer[0] =  value        & 0xFF;
    buffer[1] = (value >>  8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24);
  }

  (void) _put_bytes(&buffer[0], sizeof(buffer));
}

void StreamPeerSerial::put_64(int64_t value) {
  put_u64(static_cast<uint64_t>(value));
}

void StreamPeerSerial::put_u64(uint64_t value) {
  uint8_t buffer[8];
  if(big_endian) {
    buffer[0] = (value >> 56);
    buffer[1] = (value >> 48) & 0xFF;
    buffer[2] = (value >> 40) & 0xFF;
    buffer[3] = (value >> 32) & 0xFF;
    buffer[4] = (value >> 24) & 0xFF;
    buffer[5] = (value >> 16) & 0xFF;
    buffer[6] = (value >>  8) & 0xFF;
    buffer[7] =  value        & 0xFF;
  }
  else {
    buffer[0] =  value        & 0xFF;
    buffer[1] = (value >>  8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
    buffer[4] = (value >> 32) & 0xFF;
    buffer[5] = (value >> 40) & 0xFF;
    buffer[6] = (value >> 48) & 0xFF;
    buffer[7] = (value >> 56);
  }

  (void) _put_bytes(&buffer[0], sizeof(buffer));
}

void StreamPeerSerial::put_float(double value) {
  auto val = static_cast<float>(value);
  auto ptr = reinterpret_cast<uint32_t *>(&val);
  put_u32(*ptr);
}

void StreamPeerSerial::put_double(double value) {
  auto ptr = reinterpret_cast<uint64_t *>(&value);
  put_u64(*ptr);
}

void StreamPeerSerial::put_string(const String &value) {
  auto buffer = value.to_ascii_buffer();
  put_u32(buffer.size());
  _put_bytes(buffer.ptr(), buffer.size());
}

void StreamPeerSerial::put_utf8_string(const String &value) {
  auto buffer = value.to_utf8_buffer();
  put_u32(buffer.size());
  _put_bytes(buffer.ptr(), buffer.size());
}

void StreamPeerSerial::put_var(const Variant &value, bool full_objects) {
  PackedByteArray data = full_objects ?
                           UtilityFunctions::var_to_bytes_with_objects(value) :
                           UtilityFunctions::var_to_bytes(value);
  put_32(data.size());
  _put_bytes(data.ptr(), data.size());
}

int8_t StreamPeerSerial::get_8(){
  return static_cast<int8_t>(get_u8());
}

uint8_t StreamPeerSerial::get_u8() {
  int32_t count;
  uint8_t buffer = 0;
  (void) _get_bytes(&buffer, 1, count);
  return buffer;
}

int16_t StreamPeerSerial::get_16() {
  return static_cast<int16_t>(get_u16());
}

uint16_t StreamPeerSerial::get_u16() {
  uint8_t buffer[2];
  int32_t count, offset = 0;

  while(offset < static_cast<int32_t>(sizeof(buffer))) {
    if(_get_bytes(&buffer[offset], static_cast<int32_t>(sizeof(buffer)) - offset, count) != OK) {
      return 0;
    }

    offset += count;
  }

  if(big_endian) {
    return (buffer[0] << 8) | buffer[1];
  }
  else {
    return buffer[0] | (buffer[1] << 8);
  }
}

int32_t StreamPeerSerial::get_32() {
  return static_cast<int32_t>(get_u32());
}

uint32_t StreamPeerSerial::get_u32() {
  uint8_t buffer[4];
  int32_t count, offset = 0;

  while(offset < static_cast<int32_t>(sizeof(buffer))) {
    if(_get_bytes(&buffer[offset], static_cast<int32_t>(sizeof(buffer)) - offset, count) != OK) {
      return 0;
    }

    offset += count;
  }

  if(big_endian) {
    return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
  }
  else {
    return buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
  }
}

int64_t StreamPeerSerial::get_64() {
  return static_cast<int64_t>(get_u64());
}

uint64_t StreamPeerSerial::get_u64() {
  uint8_t buffer[8];
  int32_t count, offset = 0;

  while(offset < static_cast<int32_t>(sizeof(buffer))) {
    if(_get_bytes(&buffer[offset], static_cast<int32_t>(sizeof(buffer)) - offset, count) != OK) {
      return 0;
    }

    offset += count;
  }

  if(big_endian) {
    return (static_cast<int64_t>((buffer[0] << 24) | (buffer[1] << 16) |
                                 (buffer[2] <<  8) |  buffer[3]      ) << 32) |
            static_cast<int64_t>((buffer[4] << 24) | (buffer[5] << 16) |
                                 (buffer[6] <<  8) |  buffer[7]      );
  }
  else {
    return  static_cast<int64_t>( buffer[0]        | (buffer[1] <<  8) |
                                 (buffer[2] << 16) | (buffer[3] << 24)) |
           (static_cast<int64_t>( buffer[4]        | (buffer[5] <<  8) |
                                 (buffer[6] << 16) | (buffer[7] << 24)) << 32);
  }
}

double StreamPeerSerial::get_float() {
  auto bits = get_u32();
  auto ptr = reinterpret_cast<float *>(&bits);
  return static_cast<double>(*ptr);
}

double StreamPeerSerial::get_double() {
  auto bits = get_u64();
  auto ptr = reinterpret_cast<double *>(&bits);
  return *ptr;
}

String StreamPeerSerial::get_string(int32_t bytes) {
  if(bytes < 0) {
    bytes = get_32();
  }

  if(bytes == 0) {
    return String();
  }

  Vector<uint8_t> buffer;
  int32_t count;
  buffer.resize(bytes + 1);
  if(_get_bytes(buffer.ptrw(), bytes, count) != OK) {
    return String();
  }

  // always ensure NUL termination
  if(bytes > count) {
    buffer.ptrw()[count] = 0;
    buffer.resize(count + 1);
  }
  else {
    buffer.ptrw()[bytes] = 0;
  }

  return String(reinterpret_cast<const char *>(buffer.ptr()));
}

String StreamPeerSerial::get_utf8_string(int32_t bytes) {
  if(bytes < 0) {
    bytes = get_32();
  }

  if(bytes == 0) {
    return String();
  }

  Vector<uint8_t> buffer;
  int32_t count;
  buffer.resize(bytes + 1);
  if(_get_bytes(buffer.ptrw(), bytes, count) != OK) {
    return String();
  }

  // always ensure NUL termination
  if(bytes > count) {
    buffer.ptrw()[count] = 0;
    buffer.resize(count + 1);
  }
  else {
    buffer.ptrw()[bytes] = 0;
  }

  return String::utf8(reinterpret_cast<const char *>(buffer.ptr()), count);
}

Variant StreamPeerSerial::get_var(bool allow_objects) {
  PackedByteArray buffer;
  int32_t count;
  buffer.resize(get_32());

  if(_get_bytes(buffer.ptrw(), buffer.size(), count) == OK) {
    if(buffer.size() != count) {
      buffer.resize(count);
    }
    return allow_objects ? UtilityFunctions::bytes_to_var_with_objects(buffer) :
                           UtilityFunctions::bytes_to_var(buffer);
  }

  return Variant();
}


bool StreamPeerSerial::wait_readable() {
  serialerror_t err = serialerror_success;
  auto retVal = serial->waitReadable(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return retVal;
}


void StreamPeerSerial::wait_byte_times(size_t count) {
  serial->waitByteTimes(count);
}


PackedByteArray StreamPeerSerial::read_raw(size_t size) {
  PackedByteArray raw;
  std::vector<uint8_t> buf_temp;
  serialerror_t err = serialerror_success;
  size_t bytes_read = serial->read(buf_temp, size, &err);
  if (bytes_read > 0 && raw.resize(bytes_read) == OK) {
    memcpy(raw.ptrw(), (const char *)buf_temp.data(), bytes_read);
  }
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return raw;
}


size_t StreamPeerSerial::write_raw(const PackedByteArray &data) {
  serialerror_t err = serialerror_success;
  auto retVal = serial->write(data.ptr(), data.size(), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return retVal;
}


String StreamPeerSerial::read_line(size_t max_length, String eol, bool utf8_encoding) {
  serialerror_t err = serialerror_success;
  auto line = serial->readline(max_length, eol.utf8().get_data(), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  if(utf8_encoding) {
    String str;
    str.parse_utf8(line.c_str());
    return str;
  }

  return line.c_str();
}


PackedStringArray StreamPeerSerial::read_lines(size_t max_length, String eol, bool utf8_encoding) {
  PackedStringArray lines;
  serialerror_t err = serialerror_success;
  if (utf8_encoding) {
    for(std::string line : serial->readlines(max_length, eol.utf8().get_data())) {
      String str;
      str.parse_utf8(line.c_str());
      lines.append(str);
    }
  }
  else {
    for(std::string line : serial->readlines(max_length, eol.utf8().get_data())) {
      lines.append(line.c_str());
    }
  }

  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return lines;
}


Error StreamPeerSerial::set_port(const String &port) {
  Error result = OK;
  serialerror_t err = serialerror_success;
  serial->setPort(port.ascii().get_data(), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    switch(err) {
      case serialerror_argument:
        result = ERR_INVALID_PARAMETER;
      break;
      case serialerror_io_failed:
          result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_serial:
        result = ERR_ALREADY_IN_USE;
      break;
      default:
        result = FAILED;
      break;
    }

  }

  return result;
}


String StreamPeerSerial::get_port() const {
  serialerror_t err = serialerror_success;
  auto port = serial->getPort(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return port.c_str();
}


Error StreamPeerSerial::set_timeout(uint32_t timeout) {
	serial->setTimeout(Timeout::max(), timeout, 0, timeout, 0);
	return OK;
}


uint32_t StreamPeerSerial::get_timeout() const {
	return serial->getTimeout().read_timeout_constant;
}


Error StreamPeerSerial::set_baudrate(uint32_t baudrate) {
  serialerror_t err = serialerror_success;
  serial->setBaudrate(baudrate, &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


uint32_t StreamPeerSerial::get_baudrate() const {
  serialerror_t err = serialerror_success;
  auto baud = serial->getBaudrate(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return baud;
}


Error StreamPeerSerial::set_bytesize(ByteSize bytesize) {
  serialerror_t err = serialerror_success;
  serial->setBytesize(bytesize_t(bytesize), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


StreamPeerSerial::ByteSize StreamPeerSerial::get_bytesize() const {
  serialerror_t err = serialerror_success;
  auto sz = ByteSize(serial->getBytesize(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return sz;
}


Error StreamPeerSerial::set_parity(Parity parity) {
  serialerror_t err = serialerror_success;
  serial->setParity(parity_t(parity), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


StreamPeerSerial::Parity StreamPeerSerial::get_parity() const {
  serialerror_t err = serialerror_success;
  auto par = Parity(serial->getParity(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return par;
}


Error StreamPeerSerial::set_stopbits(StopBits stopbits) {
  serialerror_t err = serialerror_success;
  serial->setStopbits(stopbits_t(stopbits), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


StreamPeerSerial::StopBits StreamPeerSerial::get_stopbits() const {
  serialerror_t err = serialerror_success;
  auto bits = StopBits(serial->getStopbits(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return bits;
}


Error StreamPeerSerial::set_flowcontrol(FlowControl flowcontrol) {
  serialerror_t err = serialerror_success;
  serial->setFlowcontrol(flowcontrol_t(flowcontrol), &err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


StreamPeerSerial::FlowControl StreamPeerSerial::get_flowcontrol() const {
  serialerror_t err = serialerror_success;
  auto flow = FlowControl(serial->getFlowcontrol(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->_on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return flow;
}


Error StreamPeerSerial::flush() {
  if(serial->flush() != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::flush_input() {
  if(serial->flushInput() != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::flush_output() {
  if(serial->flushOutput() != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::send_break(int duration) {
  if(serial->sendBreak(duration) != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_break(bool level) {
  if(serial->setBreak(level) != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_rts(bool level) {
  if(serial->setRTS(level) != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_dtr(bool level) {
  if(serial->setDTR(level) != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
    return FAILED;
  }

  return OK;
}


bool StreamPeerSerial::wait_for_change() {
  serialerror_t err = serialerror_success;
  auto result = serial->waitForChange(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return result;
}


bool StreamPeerSerial::get_cts() {
  serialerror_t err = serialerror_success;
  auto result = serial->getCTS(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return result;
}


bool StreamPeerSerial::get_dsr() {
  serialerror_t err = serialerror_success;
  auto result = serial->getDSR(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return result;
}


bool StreamPeerSerial::get_ri() {
  serialerror_t err = serialerror_success;
  auto result = serial->getRI(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return result;
}


bool StreamPeerSerial::get_cd() {
  serialerror_t err = serialerror_success;
  auto result = serial->getCD(&err);
  if(err != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  return result;
}


String StreamPeerSerial::_to_string() const {
	Dictionary ser_info;
	ser_info["port"] = get_port();
	ser_info["baudrate"] = get_baudrate();
	ser_info["byte_size"] = get_bytesize();
	ser_info["parity"] = get_parity();
	ser_info["stop_bits"] = get_stopbits();

	return String("[StreamPeerSerial: {_}]").format(ser_info);
}

void StreamPeerSerial::_bind_methods() {
	ClassDB::bind_static_method("StreamPeerSerial", D_METHOD("list_ports"), &StreamPeerSerial::list_ports);

	ClassDB::bind_method(D_METHOD("_data_received", "data"), &StreamPeerSerial::_data_received);
	ClassDB::bind_method(D_METHOD("is_in_error"), &StreamPeerSerial::is_in_error);
	ClassDB::bind_method(D_METHOD("get_last_error"), &StreamPeerSerial::get_last_error);

	ClassDB::bind_method(D_METHOD("start_monitoring", "interval_in_usec"), &StreamPeerSerial::start_monitoring, DEFVAL(10000));
	ClassDB::bind_method(D_METHOD("stop_monitoring"), &StreamPeerSerial::stop_monitoring);

	ClassDB::bind_method(D_METHOD("open", "port"), &StreamPeerSerial::open, DEFVAL(""));
	ClassDB::bind_method(D_METHOD("is_open"), &StreamPeerSerial::is_open);
	ClassDB::bind_method(D_METHOD("close"), &StreamPeerSerial::close);

  ClassDB::bind_method(D_METHOD("put_data", "data"), &StreamPeerSerial::put_data);
  ClassDB::bind_method(D_METHOD("put_partial_data", "data"), &StreamPeerSerial::put_partial_data);
  ClassDB::bind_method(D_METHOD("get_data", "bytes"), &StreamPeerSerial::get_data);
  ClassDB::bind_method(D_METHOD("get_partial_data", "bytes"), &StreamPeerSerial::get_partial_data);
  ClassDB::bind_method(D_METHOD("get_available_bytes"), &StreamPeerSerial::get_available_bytes);
  ClassDB::bind_method(D_METHOD("set_big_endian", "enable"), &StreamPeerSerial::set_big_endian);
  ClassDB::bind_method(D_METHOD("is_big_endian_enabled"), &StreamPeerSerial::is_big_endian_enabled);
  ClassDB::bind_method(D_METHOD("put_8", "value"), &StreamPeerSerial::put_8);
  ClassDB::bind_method(D_METHOD("put_u8", "value"), &StreamPeerSerial::put_u8);
  ClassDB::bind_method(D_METHOD("put_16", "value"), &StreamPeerSerial::put_16);
  ClassDB::bind_method(D_METHOD("put_u16", "value"), &StreamPeerSerial::put_u16);
  ClassDB::bind_method(D_METHOD("put_32", "value"), &StreamPeerSerial::put_32);
  ClassDB::bind_method(D_METHOD("put_u32", "value"), &StreamPeerSerial::put_u32);
  ClassDB::bind_method(D_METHOD("put_64", "value"), &StreamPeerSerial::put_64);
  ClassDB::bind_method(D_METHOD("put_u64", "value"), &StreamPeerSerial::put_u64);
  ClassDB::bind_method(D_METHOD("put_float", "value"), &StreamPeerSerial::put_float);
  ClassDB::bind_method(D_METHOD("put_double", "value"), &StreamPeerSerial::put_double);
  ClassDB::bind_method(D_METHOD("put_string", "value"), &StreamPeerSerial::put_string);
  ClassDB::bind_method(D_METHOD("put_utf8_string", "value"), &StreamPeerSerial::put_utf8_string);
  ClassDB::bind_method(D_METHOD("put_var", "value", "full_objects"), &StreamPeerSerial::put_var, DEFVAL(false));
  ClassDB::bind_method(D_METHOD("get_8"), &StreamPeerSerial::get_8);
  ClassDB::bind_method(D_METHOD("get_u8"), &StreamPeerSerial::get_u8);
  ClassDB::bind_method(D_METHOD("get_16"), &StreamPeerSerial::get_16);
  ClassDB::bind_method(D_METHOD("get_u16"), &StreamPeerSerial::get_u16);
  ClassDB::bind_method(D_METHOD("get_32"), &StreamPeerSerial::get_32);
  ClassDB::bind_method(D_METHOD("get_u32"), &StreamPeerSerial::get_u32);
  ClassDB::bind_method(D_METHOD("get_64"), &StreamPeerSerial::get_64);
  ClassDB::bind_method(D_METHOD("get_u64"), &StreamPeerSerial::get_u64);
  ClassDB::bind_method(D_METHOD("get_float"), &StreamPeerSerial::get_float);
  ClassDB::bind_method(D_METHOD("get_double"), &StreamPeerSerial::get_double);
  ClassDB::bind_method(D_METHOD("get_string", "bytes"), &StreamPeerSerial::get_string, DEFVAL(-1));
  ClassDB::bind_method(D_METHOD("get_utf8_string", "bytes"), &StreamPeerSerial::get_utf8_string, DEFVAL(-1));
  ClassDB::bind_method(D_METHOD("get_var", "allow_objects"), &StreamPeerSerial::get_var, DEFVAL(false));

	ClassDB::bind_method(D_METHOD("wait_readable"), &StreamPeerSerial::wait_readable);
	ClassDB::bind_method(D_METHOD("wait_byte_times", "count"), &StreamPeerSerial::wait_byte_times);
	ClassDB::bind_method(D_METHOD("read_raw", "size"), &StreamPeerSerial::read_raw, DEFVAL(1));
	ClassDB::bind_method(D_METHOD("write_raw", "data"), &StreamPeerSerial::write_raw);
	ClassDB::bind_method(D_METHOD("read_line", "max_len", "eol", "utf8_encoding"), &StreamPeerSerial::read_line, DEFVAL(65535), DEFVAL("\n"), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("read_lines", "max_len", "eol", "utf8_encoding"), &StreamPeerSerial::read_lines, DEFVAL(65535), DEFVAL("\n"), DEFVAL(false));

	ClassDB::bind_method(D_METHOD("set_port", "port"), &StreamPeerSerial::set_port);
	ClassDB::bind_method(D_METHOD("get_port"), &StreamPeerSerial::get_port);
	ClassDB::bind_method(D_METHOD("set_baudrate", "baudrate"), &StreamPeerSerial::set_baudrate);
	ClassDB::bind_method(D_METHOD("get_baudrate"), &StreamPeerSerial::get_baudrate);
	ClassDB::bind_method(D_METHOD("set_timeout", "timeout"), &StreamPeerSerial::set_timeout);
	ClassDB::bind_method(D_METHOD("get_timeout"), &StreamPeerSerial::get_timeout);
	ClassDB::bind_method(D_METHOD("set_bytesize", "bytesize"), &StreamPeerSerial::set_bytesize);
	ClassDB::bind_method(D_METHOD("get_bytesize"), &StreamPeerSerial::get_bytesize);
	ClassDB::bind_method(D_METHOD("set_parity", "parity"), &StreamPeerSerial::set_parity);
	ClassDB::bind_method(D_METHOD("get_parity"), &StreamPeerSerial::get_parity);
	ClassDB::bind_method(D_METHOD("set_stopbits", "stopbits"), &StreamPeerSerial::set_stopbits);
	ClassDB::bind_method(D_METHOD("get_stopbits"), &StreamPeerSerial::get_stopbits);
	ClassDB::bind_method(D_METHOD("set_flowcontrol", "flowcontrol"), &StreamPeerSerial::set_flowcontrol);
	ClassDB::bind_method(D_METHOD("get_flowcontrol"), &StreamPeerSerial::get_flowcontrol);

	ClassDB::bind_method(D_METHOD("flush"), &StreamPeerSerial::flush);
	ClassDB::bind_method(D_METHOD("flush_input"), &StreamPeerSerial::flush_input);
	ClassDB::bind_method(D_METHOD("flush_output"), &StreamPeerSerial::flush_output);
	ClassDB::bind_method(D_METHOD("send_break", "duration"), &StreamPeerSerial::send_break);
	ClassDB::bind_method(D_METHOD("set_break", "level"), &StreamPeerSerial::set_break, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("set_rts", "level"), &StreamPeerSerial::set_rts, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("set_dtr", "level"), &StreamPeerSerial::set_dtr, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("wait_for_change"), &StreamPeerSerial::wait_for_change);
	ClassDB::bind_method(D_METHOD("get_cts"), &StreamPeerSerial::get_cts);
	ClassDB::bind_method(D_METHOD("get_dsr"), &StreamPeerSerial::get_dsr);
	ClassDB::bind_method(D_METHOD("get_ri"), &StreamPeerSerial::get_ri);
	ClassDB::bind_method(D_METHOD("get_cd"), &StreamPeerSerial::get_cd);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "port"), "set_port", "get_port");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "baudrate"), "set_baudrate", "get_baudrate");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "timeout"), "set_timeout", "get_timeout");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "bytesize", PROPERTY_HINT_ENUM, "5, 6, 7, 8"), "set_bytesize", "get_bytesize");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "parity", PROPERTY_HINT_ENUM, "None, Odd, Even, Mark, Space"), "set_parity", "get_parity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "stopbits", PROPERTY_HINT_ENUM, "1, 2, 1.5"), "set_stopbits", "get_stopbits");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "flowcontrol", PROPERTY_HINT_ENUM, "None, Software, Hardware"), "set_flowcontrol", "get_flowcontrol");

#ifndef GDEXTENSION
	ADD_PROPERTY_DEFAULT("port", "");
	ADD_PROPERTY_DEFAULT("baudrate", 9600);
	ADD_PROPERTY_DEFAULT("timeout", 0);
	ADD_PROPERTY_DEFAULT("bytesize", BYTESIZE_8);
	ADD_PROPERTY_DEFAULT("parity", PARITY_NONE);
	ADD_PROPERTY_DEFAULT("stopbits", STOPBITS_1);
	ADD_PROPERTY_DEFAULT("flowcontrol", FLOWCONTROL_NONE);
#endif

	ADD_SIGNAL(MethodInfo("got_error", PropertyInfo(Variant::STRING, "where"), PropertyInfo(Variant::STRING, "what")));
	ADD_SIGNAL(MethodInfo("opened", PropertyInfo(Variant::STRING, "port")));
	ADD_SIGNAL(MethodInfo("data_received", PropertyInfo(Variant::PACKED_BYTE_ARRAY, "data")));
	ADD_SIGNAL(MethodInfo("closed", PropertyInfo(Variant::STRING, "port")));

	BIND_ENUM_CONSTANT(BYTESIZE_5);
	BIND_ENUM_CONSTANT(BYTESIZE_6);
	BIND_ENUM_CONSTANT(BYTESIZE_7);
	BIND_ENUM_CONSTANT(BYTESIZE_8);

	BIND_ENUM_CONSTANT(PARITY_NONE);
	BIND_ENUM_CONSTANT(PARITY_ODD);
	BIND_ENUM_CONSTANT(PARITY_EVEN);
	BIND_ENUM_CONSTANT(PARITY_MARK);
	BIND_ENUM_CONSTANT(PARITY_SPACE);

	BIND_ENUM_CONSTANT(STOPBITS_1);
	BIND_ENUM_CONSTANT(STOPBITS_2);
	BIND_ENUM_CONSTANT(STOPBITS_1P5);

	BIND_ENUM_CONSTANT(FLOWCONTROL_NONE);
	BIND_ENUM_CONSTANT(FLOWCONTROL_SOFTWARE);
	BIND_ENUM_CONSTANT(FLOWCONTROL_HARDWARE);
}
