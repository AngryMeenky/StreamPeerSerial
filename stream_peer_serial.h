/*************************************************************************/
/*  serial_port.h                                                        */
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

#ifndef STREAM_PEER_SERIAL_H
#define STREAM_PEER_SERIAL_H

#ifdef GDEXTENSION
#define GDEXT_OVERRIDE
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/classes/stream_peer.hpp>
#include <godot_cpp/variant/builtin_types.hpp>

using namespace godot;
#else
#define GDEXT_OVERRIDE override
#include "core/io/stream_peer.h"
#include "core/string/ustring.h"
#include "core/templates/vector.h"
#include "core/variant/array.h"
#include "core/variant/dictionary.h"
#endif

#include "serial/serial.h"

#include <atomic>
#include <thread>

using namespace serial;

class StreamPeerSerial : public StreamPeer {
	GDCLASS(StreamPeerSerial, StreamPeer);

	static void _thread_func(void *p_user_data);

	Serial *serial;
	int monitoring_interval = 10000;
	std::atomic<bool> fine_working = false;
	std::atomic<bool> monitoring_should_exit = true;
	std::thread thread;

	String error_message = "";

	void _data_received(const PackedByteArray &buf);

public:
	enum ByteSize {
		BYTESIZE_5 = fivebits,
		BYTESIZE_6 = sixbits,
		BYTESIZE_7 = sevenbits,
		BYTESIZE_8 = eightbits,
	};
	enum Parity {
		PARITY_NONE = parity_none,
		PARITY_ODD = parity_odd,
		PARITY_EVEN = parity_even,
		PARITY_MARK = parity_mark,
		PARITY_SPACE = parity_space,
	};
	enum StopBits {
		STOPBITS_1 = stopbits_one,
		STOPBITS_2 = stopbits_two,
		STOPBITS_1P5 = stopbits_one_point_five,
	};
	enum FlowControl {
		FLOWCONTROL_NONE = flowcontrol_none,
		FLOWCONTROL_SOFTWARE = flowcontrol_software,
		FLOWCONTROL_HARDWARE = flowcontrol_hardware,
	};

  StreamPeerSerial(const String &port = "",
			uint32_t baudrate = 9600,
			uint32_t timeout = 0,
			ByteSize bytesize = BYTESIZE_8,
			Parity parity = PARITY_NONE,
			StopBits stopbits = STOPBITS_1,
			FlowControl flowcontrol = FLOWCONTROL_NONE);

	~StreamPeerSerial();

	static Dictionary list_ports();

 	Error put_data(const uint8_t *p_data, int p_bytes) GDEXT_OVERRIDE;
	Error put_partial_data(const uint8_t *p_data, int p_bytes, int &r_sent) GDEXT_OVERRIDE;

	Error get_data(uint8_t *p_buffer, int p_bytes) GDEXT_OVERRIDE;
	Error get_partial_data(uint8_t *p_buffer, int p_bytes, int &r_received) GDEXT_OVERRIDE;

	int get_available_bytes() const GDEXT_OVERRIDE;

	bool is_in_error() { return is_open() && !fine_working; }
	inline String get_last_error() { return error_message; }
	void _on_error(const String &where, const String &what);

	Error start_monitoring(uint64_t interval_in_usec = 10000);
	void stop_monitoring();

	Error open(String port = "");

	bool is_open() const;

	void close();

	bool wait_readable();

	void wait_byte_times(size_t count);

	PackedByteArray read_raw(size_t size = 1);

	size_t write_raw(const PackedByteArray &data);

	String read_line(size_t size = 65535, String eol = "\n", bool utf8_encoding = false);
	PackedStringArray read_lines(size_t size = 65535, String eol = "\n", bool utf8_encoding = false);

	Error set_port(const String &port);

	String get_port() const;

	Error set_timeout(uint32_t timeout);

	uint32_t get_timeout() const;

	Error set_baudrate(uint32_t baudrate);

	uint32_t get_baudrate() const;

	Error set_bytesize(ByteSize bytesize);

	ByteSize get_bytesize() const;

	Error set_parity(Parity parity);

	Parity get_parity() const;

	Error set_stopbits(StopBits stopbits);

	StopBits get_stopbits() const;

	Error set_flowcontrol(FlowControl flowcontrol);

	FlowControl get_flowcontrol() const;

	Error flush();

	Error flush_input();

	Error flush_output();

	Error send_break(int duration);

	Error set_break(bool level = true);

	Error set_rts(bool level = true);

	Error set_dtr(bool level = true);

	bool wait_for_change();

	bool get_cts();

	bool get_dsr();

	bool get_ri();

	bool get_cd();

protected:
	String _to_string() const;

	static void _bind_methods();
};

VARIANT_ENUM_CAST(StreamPeerSerial::ByteSize);
VARIANT_ENUM_CAST(StreamPeerSerial::Parity);
VARIANT_ENUM_CAST(StreamPeerSerial::StopBits);
VARIANT_ENUM_CAST(StreamPeerSerial::FlowControl);

#endif // STREAM_PEER_SERIAL_H
