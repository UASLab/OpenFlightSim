var dialog = gui.Dialog.new("/sim/gui/dialogs/rascal/config/dialog",
                            "Dialogs/config.xml");

var last_time = 0.0;


var main_loop = func {
    var time = getprop("/sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    update_aura( dt );

    settimer(main_loop, 0);
}


setlistener("/sim/signals/fdm-initialized",
  	func {
  		  main_loop();
  	}
);
