# Laser 350/500/700 (Video Technology) emulator

The Laser 350/500/700 is a family of home computers based on the Z80 CPU
made by Video Technology in 1985. `laser500emu` emulates these computers in 
a web browser. 

HOW TO USE
==========

To run the emulator, simply open to the link: 
[nippur72.github.io/laser500emu](https://nippur72.github.io/laser500emu/). 

You can also clone the repo and run locally, just open the file `index.html`.

The emulator is written in JavaScript and runs fine in Chrome and FireFox. 

KEYBOARD
========
Currently only the italian keyboard layout is supported.

Special keys:
- `Pause`: Laser's Reset key (it acts like a more powerful CTRL+Break)
- `Alt`+`R`: alternate for Reset key
- `Alt`+`P`: power on/off the machine
- `Home`: HOME key
- `Shift`+`Home`: CLS key (clear screen)
- `End`: DEL LINE key
- `Del`: DEL key 
- `§`: GRAPH key 
- Cursor Left: rewinds and plays the tape
- Cursor Up: stops the tape
- Numpad + `0` + right `Ctrl`: Joystick 1

EMULATION SPEED
===============

If emulation is slower than expected you can check how much CPU load 
the emulator is consuming. Open the JavaScript console (`F12`) and type `info()`
```
> info()
frame rendering: 2.9 ms, load=14.5 %
```
Frame rendering should be less than 20 ms which is the PAL framerate 
it is being emulated.

AUTOSAVE
========
The machine state will be automatically saved and restored when you 
close and reopen the browser, taking you back where you left it.

If you want to start from scratch, press `ALT+P` to simulate power on/off.

EMULATOR FEATURES
=================
- accurate to the scanline level (changing video mode will reflect next scanline)
- all graphic modes are emulated 
- sound is accurate but lags due to browser latency
- old CRT look (simulated scanlines), 50 fps as in PAL standard
- cassette output is redirected to PC speakers (can be recorded and played back on a real machine)
- printer is emulated by redirecting it to the JavaScript console (F12 on the browser)
- ROM Basic 3.0 
- Joystick 1 emulated as NUMPAD + RIGHT CTRL (fire1) + NUMPAD0 (fire2)
- Floppy Disk Drive NOT emulated (we miss the ROM DOS Basic 1.0, if you have it please contact me)
- Keyboard not perfect yet, and with italian layout only
- ALT+R or CTRL+BREAK is reset key, ALT+P is power on button
- Drag & drop binary files to load them on the emulator
- automatically saves and restores the state of the emulator

There are also a bunch of commands that you can run from the JavaScript console (F12)
to load or save programs. Saved programs are seen as a downloaded file in the browser.

Programs are also stored on the internal browser storage (HTML5 `localStorage`), simulating
a sort of disk drive.

RESOURCES
=========

The `laser500emu` repo serves also as a container for Laser 500 software 
and documentation that I have collected along the way, make sure to have a look at:

- [software](https://github.com/nippur72/laser500emu/tree/gh-pages/software)
- [docs](https://github.com/nippur72/laser500emu/tree/gh-pages/docs)

Other resources about the Laser 350/500/700 computers are:

- Bonstra's GitHub repo [laser500-doc](https://github.com/Bonstra/laser500-doc) by 
- a dedicated [Facebook group](https://www.facebook.com/groups/263150584310074) 
- a forum thread on [AtariAge](http://atariage.com/forums/topic/187667-any-info-on-video-technology-laser-500-computer/page-1).
