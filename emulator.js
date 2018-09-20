"use strict";

// TODO almost exact cycles
// TODO sound doesn't work in Mozilla
// TODO load WAV from cassette or mic
// TODO javascript debugger, halt
// TODO laser 350
// TODO laser 700
// TODO rename ram1, ram2 to page
// TODO laser 200 family? study vzem
// TODO Z80js, port in ES6 then webassembly
// TODO draw in webassembly
// TODO caplock key / led ?
// TODO visual/sound display of activity
// TODO wrap in electron app
// TODO save to cloud / fetch()
// TODO verificare range indirizzi di cassette_bit 
// TODO some way of pasting text
// TODO interrupt routine test
// TODO check colors with real hardware
// TODO options window (modal)
// TODO options from URL &parameters
// TODO investigate port 13h reads
// TODO emulate floppy
// TODO build of CP/M ?
// TODO draw keyboard for mobile

// *** laser 500 hardware ***

// rom1,rom2 are defined in roms.js
const ram1     = new Uint8Array(16384); // page 4
const ram2     = new Uint8Array(16384); // page 5
const ram3     = new Uint8Array(16384); // page 6
const videoram = new Uint8Array(16384); // page 7
const banks    = new Uint8Array(4);

// page 3 only on laser 350 
const page3    = new Uint8Array(16384);

// makes page 3 respond as 0xFF as in real hardware
page3.forEach((e,i)=>page3[i]=0xFF); 

let cassette_bit_in = 1; 
let cassette_bit_out = 0; 
let vdc_graphic_mode_enabled = 0;
let vdc_graphic_mode_number = 0;
let vdc_page_7 = 0;
let vdc_text80_enabled = 0;
let vdc_text80_foreground = 0;
let vdc_text80_background = 0;
let vdc_border_color = 0;
let speaker_A = 0;
let speaker_B = 0;
let joy0 = 255;
let joy1 = 255;

let cpu = new Z80({ mem_read, mem_write, io_read, io_write });

/******************/

const frameRate = 50; // 50 Hz PAL standard
const frameDuration = 1000/frameRate; // duration of 1 frame in msec
const cpuSpeed = 3694700; // Z80 speed 3.6947 MHz (NEC D780c)
const cyclesPerFrame = (cpuSpeed / frameDuration) / 3.5; // 
const cyclesPerLine = 190;
const cpuSampleRate = cyclesPerLine * TOTAL_SCANLINES * frameRate;

let stopped = false; // allows to stop/resume the emulation

// PAL Standard: 720 x 576

// 192 righe video + 96 bordo (48 sopra e 48 sotto) = 192+96 = 288 ; x2 = 576

let frames = 0;
let nextFrameTime = 0;
let averageFrameTime = 0;

let cycle = 0;
let cycles = 0;

// scanline version
function renderLines(nlines, hidden) {
   for(let t=0; t<nlines; t++) {
      // draw video
      if(!hidden) drawFrame_y();

      // run cpu
      while(true) {
         const elapsed = cpu.run_instruction();
         cycle += elapsed;
         cycles += elapsed;

         writeAudioSamples(elapsed);
         
         if(cycle>=cyclesPerLine) {
            cycle-=cyclesPerLine;
            break;            
         }
      } 
   }
}

/*
// versione cycle exact
function renderAllLines() {   
   while(raster_y < SCREEN_H) 
   {
      const elapsed = cpu.run_instruction();
      writeAudioSamples(elapsed);
      cycle += elapsed * 720;
      while(cycle > 0) {
         drawEight();
         cycle -= 1520;
      }
      //console.log(raster_y, SCREEN_H)   ;
      //if(zz++ % 30590 === 0) break;
   }
   
   cpu.interrupt(false, 0);                  

   
   //cycle += PAL_HIDDEN_LINES_VERY_BOTTOM * cyclesPerLine;
   ////while(cycle > 0) 
   //{
   //   const elapsed = cpu.run_instruction();
   //   writeAudioSamples(elapsed);
   //   cycle -= elapsed;
   //}
      
   raster_y = 0;
}
*/

function renderAllLines() {
   renderLines(HIDDEN_SCANLINES_TOP, true);         // hidden lines at top
   renderLines(SCREEN_H, false);                    // screen
   renderLines(HIDDEN_SCANLINES_BOTTOM, true);      // hidden lines at bottom   
   cpu.interrupt(false, 0);                         // generate VDC interrupt
   renderLines(PAL_HIDDEN_LINES_VERY_BOTTOM, true); // hidden lines at bottom
}

let nextFrame;
function oneFrame() {   
   const startTime = new Date().getTime();      

   if(nextFrame === undefined) nextFrame = startTime;

   nextFrame = nextFrame + 20; // 20ms, 50Hz  

   renderAllLines();   

   const now = new Date().getTime();
   const elapsed = now - startTime;
   averageFrameTime = averageFrameTime * 0.99 + elapsed * 0.01;

   let time_out = nextFrame - now;
   if(time_out < 0) {
      time_out = 0;
      nextFrame = undefined;      
   }
   if(!stopped) setTimeout(()=>oneFrame(), time_out);   
}

/*********************************************************************************** */

const audioBufferSize = 16384; // enough to hold more than one frame time
const audioBuffer = new Float32Array(audioBufferSize);

let samplePtr = 0; // high speed pointer used to downsample
let audioPtr = 0;  // pointer at audio frequency
let audioPlayPtr = 0;
let audioPtr_unclipped = 0;
let audioPlayPtr_unclipped = 0;

let initialSync = false;

function writeAudioSamples(n) {
   samplePtr += (n * sampleRate);
   if(samplePtr > cpuSampleRate) {      
      const s = (speaker_A ? -0.5 : 0.0) + (cassette_bit_out ? 0.5 : 0.0);
      samplePtr -= cpuSampleRate;
      audioBuffer[audioPtr++] = s;
      audioPtr = audioPtr % audioBufferSize;
      audioPtr_unclipped++;
   }      
}

let audioContext = new (window.AudioContext || window.webkitAudioContext)();
const bufferSize = 2048;
const sampleRate = audioContext.sampleRate;
var speakerSound = audioContext.createScriptProcessor(bufferSize, 1, 1);

speakerSound.onaudioprocess = function(e) {
   if(audioPtr_unclipped < (audioPlayPtr_unclipped + bufferSize)) {
      // console.warn(`audio buffer not filled: ${audioPtr_unclipped} < ${audioPlayPtr_unclipped + bufferSize}, behind ${audioPtr_unclipped - (audioPlayPtr_unclipped + bufferSize)}`);
      return; 
   }   

   const output = e.outputBuffer.getChannelData(0);
   for(let i=0; i<bufferSize; i++) {
      const audio = audioBuffer[audioPlayPtr++];
      audioPlayPtr = audioPlayPtr % audioBufferSize;
      audioPlayPtr_unclipped++;
      output[i] = audio;
    }        
}

speakerSound.connect(audioContext.destination);

function ss() {
   speakerSound.disconnect(audioContext.destination);
}

/*********************************************************************************** */

// prints welcome message on the console
welcome();

// try to restore previous state, if any
restoreState();

// starts drawing frames
oneFrame();

