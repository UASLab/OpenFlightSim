socat -d -d PTY,link=ptySimSoc,rawer tcp-listen:59600,reuseaddr,fork; stty sane;
