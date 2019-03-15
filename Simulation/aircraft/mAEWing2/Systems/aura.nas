var update_aura = func( dt ) {

  var ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  if ( ap_enable == nil ) {
    props.globals.initNode("/aura-uas/settings/ap-enable", 0, "BOOL", 1);
    ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  }

  if ( ap_enable.getBoolValue() ) {
  	setprop( "/fdm/jsbsim/fcs/cmdTE1L_ext_rad", getprop("/aura-uas/act/cmdAct1_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE1R_ext_rad", getprop("/aura-uas/act/cmdAct2_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE2L_ext_rad", getprop("/aura-uas/act/cmdAct3_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE2R_ext_rad", getprop("/aura-uas/act/cmdAct4_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE3L_ext_rad", getprop("/aura-uas/act/cmdAct5_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE3R_ext_rad", getprop("/aura-uas/act/cmdAct6_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE4L_ext_rad", getprop("/aura-uas/act/cmdAct7_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE4R_ext_rad", getprop("/aura-uas/act/cmdAct8_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE5L_ext_rad", getprop("/aura-uas/act/cmdAct9_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdTE5R_ext_rad", getprop("/aura-uas/act/cmdAct10_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdLEL_ext_rad", getprop("/aura-uas/act/cmdAct11_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdLER_ext_rad", getprop("/aura-uas/act/cmdAct12_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdMotor_ext_nd", getprop("/aura-uas/act/cmdAct13_rad") );
  }
}
