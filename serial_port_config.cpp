#include "serial_port_config.h"


SerialPortConfig::SerialPortConfig() {
  // on successful creation default to an arduino compatible setup
  if(sp_new_config(&_config) == SP_OK) {
    // 8N1 without flow control
    sp_set_config_bits(_config, 8);
    sp_set_config_parity(_config, SP_PARITY_NONE);
    sp_set_config_stopbits(_config, 1);
    sp_set_config_flowcontrol(_config, SP_FLOWCONTROL_NONE);
  }
}


SerialPortConfig::~SerialPortConfig() {
  if(_config) {
    replace(nullptr);
  }
}


sp_port_config *SerialPortConfig::swap(sp_port_config *config) {
  sp_port_config *tmp = _config;
  _config = config;
  return tmp;
}


void SerialPortConfig::replace(sp_port_config *config) {
  if((config = swap(config)) != nullptr) {
    sp_free_config(config);
  }
}


bool SerialPortConfig::has_baudrate() const {
  return get_baudrate() > 0;
}


int64_t SerialPortConfig::get_baudrate() const {
  int rate = -1;
  return static_cast<int64_t>(sp_get_config_baudrate(_config, &rate) == SP_OK ? rate : -1);
}


Error SerialPortConfig::set_baudrate(int64_t rate) {
  return sp_set_config_baudrate(_config, static_cast<int>(rate)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_data_bits() const {
  return get_data_bits() > 0;
}


int64_t SerialPortConfig::get_data_bits() const {
  int bits = -1;
  return static_cast<int64_t>(sp_get_config_bits(_config, &bits) == SP_OK ? bits : -1);
}


Error SerialPortConfig::set_data_bits(int64_t bits) {
  return sp_set_config_bits(_config, static_cast<int>(bits)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_stop_bits() const {
  return get_stop_bits() >= 0;
}


int64_t SerialPortConfig::get_stop_bits() const {
  int bits = -1;
  return static_cast<int64_t>(sp_get_config_stopbits(_config, &bits) == SP_OK ? bits : -1);
}


Error SerialPortConfig::set_stop_bits(int64_t bits) {
  return sp_set_config_stopbits(_config, static_cast<int>(bits)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_parity() const {
  return get_parity() != INVALID_PARITY;
}


SerialPortConfig::Parity SerialPortConfig::get_parity() const {
  sp_parity parity = SP_PARITY_INVALID;
  return static_cast<Parity>(sp_get_config_parity(_config, &parity) == SP_OK ? parity : SP_PARITY_INVALID);
}


Error SerialPortConfig::set_parity(Parity parity) {
  return sp_set_config_parity(_config, static_cast<sp_parity>(parity)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_rts() const {
  return get_rts() != INVALID_RTS;
}


SerialPortConfig::Rts SerialPortConfig::get_rts() const {
  sp_rts rts = SP_RTS_INVALID;
  return static_cast<Rts>(sp_get_config_rts(_config, &rts) == SP_OK ? rts : SP_RTS_INVALID);
}


Error SerialPortConfig::set_rts(Rts rts) {
  return sp_set_config_rts(_config, static_cast<sp_rts>(rts)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_cts() const {
  return get_cts() != INVALID_CTS;
}


SerialPortConfig::Cts SerialPortConfig::get_cts() const {
  sp_cts cts = SP_CTS_INVALID;
  return static_cast<Cts>(sp_get_config_cts(_config, &cts) == SP_OK ? cts : SP_CTS_INVALID);
}


Error SerialPortConfig::set_cts(Cts cts) {
  return sp_set_config_cts(_config, static_cast<sp_cts>(cts)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_dtr() const {
  return get_dtr() != INVALID_DTR;
}


SerialPortConfig::Dtr SerialPortConfig::get_dtr() const {
  sp_dtr dtr = SP_DTR_INVALID;
  return static_cast<Dtr>(sp_get_config_dtr(_config, &dtr) == SP_OK ? dtr : SP_DTR_INVALID);
}


Error SerialPortConfig::set_dtr(Dtr dtr) {
  return sp_set_config_dtr(_config, static_cast<sp_dtr>(dtr)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_dsr() const {
  return get_dsr() != INVALID_DSR;
}


SerialPortConfig::Dsr SerialPortConfig::get_dsr() const {
  sp_dsr dsr = SP_DSR_INVALID;
  return static_cast<Dsr>(sp_get_config_dsr(_config, &dsr) == SP_OK ? dsr : SP_DSR_INVALID);
}


Error SerialPortConfig::set_dsr(Dsr dsr) {
  return sp_set_config_dsr(_config, static_cast<sp_dsr>(dsr)) == SP_OK ? OK : FAILED;
}


bool SerialPortConfig::has_xon_xoff() const {
  return get_xon_xoff() != INVALID_XONXOFF;
}


SerialPortConfig::XonXoff SerialPortConfig::get_xon_xoff() const {
  sp_xonxoff x = SP_XONXOFF_INVALID;
  return static_cast<XonXoff>(sp_get_config_xon_xoff(_config, &x) == SP_OK ? x : SP_XONXOFF_INVALID);
}


Error SerialPortConfig::set_xon_xoff(XonXoff x) {
  return sp_set_config_xon_xoff(_config, static_cast<sp_xonxoff>(x)) == SP_OK ? OK : FAILED;
}


Error SerialPortConfig::set_flow_control(FlowControl f) {
  return sp_set_config_flowcontrol(_config, static_cast<sp_flowcontrol>(f)) == SP_OK ? OK : FAILED;
}


void SerialPortConfig::_bind_methods() {
  ClassDB::bind_method(D_METHOD("has_baudrate"),                 &SerialPortConfig::has_baudrate);
  ClassDB::bind_method(D_METHOD("get_baudrate"),                 &SerialPortConfig::get_baudrate);
  ClassDB::bind_method(D_METHOD("set_baudrate",     "rate"),     &SerialPortConfig::set_baudrate);
  ClassDB::bind_method(D_METHOD("has_data_bits"),                &SerialPortConfig::has_data_bits);
  ClassDB::bind_method(D_METHOD("get_data_bits"),                &SerialPortConfig::get_data_bits);
  ClassDB::bind_method(D_METHOD("set_data_bits",    "bits"),     &SerialPortConfig::set_data_bits);
  ClassDB::bind_method(D_METHOD("has_stop_bits"),                &SerialPortConfig::has_stop_bits);
  ClassDB::bind_method(D_METHOD("get_stop_bits"),                &SerialPortConfig::get_stop_bits);
  ClassDB::bind_method(D_METHOD("set_stop_bits",    "bits"),     &SerialPortConfig::set_stop_bits);
  ClassDB::bind_method(D_METHOD("has_parity"),                   &SerialPortConfig::has_parity);
  ClassDB::bind_method(D_METHOD("get_parity"),                   &SerialPortConfig::get_parity);
  ClassDB::bind_method(D_METHOD("set_parity",       "parity"),   &SerialPortConfig::set_parity);
  ClassDB::bind_method(D_METHOD("has_rts"),                      &SerialPortConfig::has_rts);
  ClassDB::bind_method(D_METHOD("get_rts"),                      &SerialPortConfig::get_rts);
  ClassDB::bind_method(D_METHOD("set_rts",          "rts"),      &SerialPortConfig::set_rts);
  ClassDB::bind_method(D_METHOD("has_cts"),                      &SerialPortConfig::has_cts);
  ClassDB::bind_method(D_METHOD("get_cts"),                      &SerialPortConfig::get_cts);
  ClassDB::bind_method(D_METHOD("set_cts",          "cts"),      &SerialPortConfig::set_cts);
  ClassDB::bind_method(D_METHOD("has_dtr"),                      &SerialPortConfig::has_dtr);
  ClassDB::bind_method(D_METHOD("get_dtr"),                      &SerialPortConfig::get_dtr);
  ClassDB::bind_method(D_METHOD("set_dtr",          "dtr"),      &SerialPortConfig::set_dtr);
  ClassDB::bind_method(D_METHOD("has_dsr"),                      &SerialPortConfig::has_dsr);
  ClassDB::bind_method(D_METHOD("get_dsr"),                      &SerialPortConfig::get_dsr);
  ClassDB::bind_method(D_METHOD("set_dsr",          "dsr"),      &SerialPortConfig::set_dsr);
  ClassDB::bind_method(D_METHOD("has_xon_xoff"),                 &SerialPortConfig::has_xon_xoff);
  ClassDB::bind_method(D_METHOD("get_xon_xoff"),                 &SerialPortConfig::get_xon_xoff);
  ClassDB::bind_method(D_METHOD("set_xon_xoff",     "xon_xoff"), &SerialPortConfig::set_xon_xoff);
  ClassDB::bind_method(D_METHOD("set_flow_control", "control"),  &SerialPortConfig::set_flow_control);

  ADD_PROPERTY(PropertyInfo(Variant::INT, "baudrate"),  "set_baudrate",  "get_baudrate");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "data_bits"), "set_data_bits", "get_data_bits");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "stop_bits"), "set_stop_bits", "get_stop_bits");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "parity", PROPERTY_HINT_ENUM_SUGGESTION, "None,Odd,Even,Mark,Space"), "set_parity", "get_parity");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "rts", PROPERTY_HINT_ENUM_SUGGESTION, "Off,On,Flow Control"), "set_rts", "get_rts");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "cts", PROPERTY_HINT_ENUM_SUGGESTION, "Ignore,Flow Control"), "set_cts", "get_cts");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "dtr", PROPERTY_HINT_ENUM_SUGGESTION, "Off,On,Flow Control"), "set_dtr", "get_dtr");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "dsr", PROPERTY_HINT_ENUM_SUGGESTION, "Ignore,Flow Control"), "set_dsr", "get_dsr");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "xon_xoff", PROPERTY_HINT_ENUM_SUGGESTION, "DIsabled,In,Out,InOut"), "set_xon_xoff", "get_xon_xoff");

  BIND_ENUM_CONSTANT(INVALID_PARITY);
  BIND_ENUM_CONSTANT(NO_PARITY);
  BIND_ENUM_CONSTANT(ODD_PARITY);
  BIND_ENUM_CONSTANT(EVEN_PARITY);
  BIND_ENUM_CONSTANT(MARK_PARITY);
  BIND_ENUM_CONSTANT(SPACE_PARITY);

  BIND_ENUM_CONSTANT(INVALID_RTS);
  BIND_ENUM_CONSTANT(RTS_OFF);
  BIND_ENUM_CONSTANT(RTS_ON);
  BIND_ENUM_CONSTANT(RTS_FLOW);

  BIND_ENUM_CONSTANT(INVALID_CTS);
  BIND_ENUM_CONSTANT(CTS_IGNORE);
  BIND_ENUM_CONSTANT(CTS_FLOW);

  BIND_ENUM_CONSTANT(INVALID_DTR);
  BIND_ENUM_CONSTANT(DTR_OFF);
  BIND_ENUM_CONSTANT(DTR_ON);
  BIND_ENUM_CONSTANT(DTR_FLOW);

  BIND_ENUM_CONSTANT(INVALID_DSR);
  BIND_ENUM_CONSTANT(DSR_IGNORE);
  BIND_ENUM_CONSTANT(DSR_FLOW);

  BIND_ENUM_CONSTANT(INVALID_XONXOFF);
  BIND_ENUM_CONSTANT(XONXOFF_DISABLED);
  BIND_ENUM_CONSTANT(XONXOFF_IN);
  BIND_ENUM_CONSTANT(XONXOFF_OUT);
  BIND_ENUM_CONSTANT(XONXOFF_INOUT);

  BIND_ENUM_CONSTANT(FLOW_NONE);
  BIND_ENUM_CONSTANT(FLOW_XONXOFF);
  BIND_ENUM_CONSTANT(FLOW_RTSCTS);
  BIND_ENUM_CONSTANT(FLOW_DTRDSR);
}
