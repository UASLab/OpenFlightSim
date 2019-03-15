var update_aura = func( dt ) {

  var ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  if ( ap_enable == nil ) {
    props.globals.initNode("/aura-uas/settings/ap-enable", 0, "BOOL", 1);
    ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
  }

  if ( ap_enable.getBoolValue() ) {
  	setprop( "/fdm/jsbsim/fcs/cmdAilL_ext_rad", getprop("/aura-uas/act/cmdAct1_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdAilR_ext_rad", getprop("/aura-uas/act/cmdAct2_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdElev_ext_rad", getprop("/aura-uas/act/cmdAct3_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdRud_ext_rad" , getprop("/aura-uas/act/cmdAct4_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdFlapL_ext_rad", getprop("/aura-uas/act/cmdAct5_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdFlapR_ext_rad", getprop("/aura-uas/act/cmdAct6_rad") );
  	setprop( "/fdm/jsbsim/fcs/cmdMotor_ext_nd", getprop("/aura-uas/act/cmdAct7_rad") );
  }
}
