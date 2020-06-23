socat -d -d tcp-listen:8000,reuseaddr,fork PTY,link=ptySimSoc,rawer; stty sane;
