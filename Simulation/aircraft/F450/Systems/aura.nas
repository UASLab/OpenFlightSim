var update_aura = func( dt ) {

  var ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  if ( ap_enable == nil ) {
    props.globals.initNode("/aura-uas/settings/ap-enable", 0, "BOOL", 1);
    ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  }

  if ( ap_enable.getBoolValue() ) {
  	setprop( "/fdm/jsbsim/fcs/cmdMotorFR_ext_nd", getprop("/aura-uas/act/cmdAct1_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdMotorAL_ext_nd", getprop("/aura-uas/act/cmdAct2_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdMotorFL_ext_nd", getprop("/aura-uas/act/cmdAct3_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdMotorAR_ext_nd" , getprop("/aura-uas/act/cmdAct4_rad") );
  }
}
