// handles interaction between browser and emulation 

function onResize(e) {
   const canvas = document.getElementById("canvas");
   const aspect = 1.55;
   if(window.innerWidth > (window.innerHeight*aspect))
   {
      canvas.style.width  = `${aspect*100}vmin`;
      canvas.style.height = "100vmin";
   }
   else if(window.innerWidth > window.innerHeight)
   {
      canvas.style.width  = "100vmax";
      canvas.style.height = `${(1/aspect)*100}vmax`;
   }
   else
   {
      canvas.style.width  = "100vmin";
      canvas.style.height = `${(1/aspect)*100}vmin`;
   }   
}

function goFullScreen() 
{
        if(canvas.webkitRequestFullscreen !== undefined) canvas.webkitRequestFullscreen();
   else if(canvas.mozRequestFullScreen !== undefined) canvas.mozRequestFullScreen();      
   onResize();
}

window.addEventListener("resize", onResize);
window.addEventListener("dblclick", goFullScreen);

onResize();

// **** save state on close ****

window.onbeforeunload = function(e) {
   saveState();   
 };

const fdd0Panel = document.getElementById('fdd0');
// **** FDD panels *****
function fdd_update_drvsel(idx, state) {
   var indicators;
   if (idx == 0) {
      indicators = fdd0Panel.getElementsByClassName("indicator");
   } else {
      return;
   }
   for (let i of indicators) {
      if (i.getAttribute("name") != "drvsel")
         continue;
      const led = i.getElementsByClassName("led");
      if (led.length == 0)
         continue;
      led[0].setAttribute("state", state ? "on" : "off");
   };
}

function fdd_update_track(idx, track) {
   var indicators;
   if (idx == 0) {
      indicators = fdd0Panel.getElementsByClassName("indicator");
   } else {
      return;
   }
   for (let i of indicators) {
      if (i.getAttribute("name") != "trackno")
         continue;
      const span = i.getElementsByClassName("trackno");
      if (span.length == 0)
         continue;
      span[0].textContent = "" + track + "";
   };
}

// **** drag & drop ****

const dropZone = document.getElementById('canvas');
// Optional.   Show the copy icon when dragging over.  Seems to only work for chrome.
dropZone.addEventListener('dragover', function(e) {
   e.stopPropagation();
   e.preventDefault();
   e.dataTransfer.dropEffect = 'copy';
});

// Get file data on drop
dropZone.addEventListener('drop', e => {
   e.stopPropagation();
   e.preventDefault();
   const files = e.dataTransfer.files; // Array of all files

   for(let i=0, file; file=files[i]; i++) {                   
      const reader = new FileReader();
      reader.onload = e2 => droppedFile(file.name, e2.target.result);
      reader.readAsArrayBuffer(file); 
   }
});

fdd0Panel.addEventListener('dragover', function(e) {
    e.stopPropagation();
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
    e.currentTarget.setAttribute("drag-target", "yes");
});

fdd0Panel.addEventListener('dragleave', function(e) {
    e.stopPropagation();
    e.preventDefault();
    e.currentTarget.setAttribute("drag-target", "no");
});

fdd0Panel.addEventListener('drop', e => {
   const panel = e.currentTarget;
   e.stopPropagation();
   e.preventDefault();
   panel.setAttribute("drag-target", "no");
   if (e.dataTransfer.files.length != 1) {
      console.warn("Expected only 1 dropped file, got " + e.dataTransfer.files.length + ".");
      return;
   }
   const file = e.dataTransfer.files[0];

   const reader = new FileReader();
   reader.onload = e2 => {
      if (!e2.lengthComputable) {
         console.error("Cannot determine image size.");
         e2.target.abort();
         return;
      }
      if (e2.total > 10000000) {
         console.error("File is too large.");
         e2.target.abort();
         return;
      }
      // File loaded
      if (e2.loaded == e2.total) {
         if (drives[0].loadDisk(e2.target.result)) {
            panel.getElementsByClassName('filename')[0].textContent = file.name;
         } else {
            panel.getElementsByClassName('filename')[0].textContent = "(invalid image)";
         }
      }
   };
   reader.onabort = e2 => {
      panel.getElementsByClassName('filename')[0].textContent = "(failed to load image file)";
   };
   reader.readAsArrayBuffer(file);
});

function droppedFile(outName, bytes) {   

   const ext = /\.wav$/i;
   if(ext.test(outName)) {
      // WAV files
      console.log("WAV file dropped");
      const info = decodeSync(bytes);
      tapeSampleRate = info.sampleRate;
      console.log(info.channelData);
      tapeBuffer = info.channelData[0];
      tapeLen = tapeBuffer.length;
      tapePtr = 0;
      tapeHighPtr = 0;
      return;
   }

   const saveObject = {
      name: outName,
      bytes: Array.from(new Uint8Array(bytes)),
      start: 0x8995,
      type: "bin"
   };
            
   window.localStorage.setItem(`laser500/${outName}`, JSON.stringify(saveObject));
         
   crun(outName);         
}

// **** welcome message ****

function welcome() {
   console.info("Welcome to the Video Technology Laser 500 emulator");
   console.info("To load files into the emulator, drag & drop a file over the screen");
   console.info("From the console you can use the following functions:");
   console.info("");
   console.info("    csave(name[,start,end])");
   console.info("    crun(name)");
   console.info("    cload(name)");
   console.info("    cdir()");
   console.info("    cdel(name)");
   console.info("    info()");
   console.info("    stop()");
   console.info("    go()");
   console.info("    power()");
   console.info("");
   console.info("Loaded and saved files are also stored permanently on the browser memory");
   console.info("Printer is emulated by printing on the JavaScript console (here)");
   console.info("Reset key is Pause or Ctrl+Break or Alt+R");
   console.info("Power on/off Alt+P");
   console.info("Currently only italian keyboard is mapped.");
   console.info("");
   console.info("Emulation is still in development");
   console.info("");
   console.info("");
}

function getQueryStringObject() {
   let a = window.location.search.split("&");
   let o = a.reduce((o, v) =>{
      var kv = v.split("=");
      kv[0] = kv[0].replace("?", "");
      o[kv[0]] = kv[1];
      return o;
      },{}
   );
   return o;
}

function parseQueryStringCommands() {
   const cmd = getQueryStringObject();

   if(cmd.load !== undefined) {
      name = cmd.load;      
      fetchProgramAll(name);            
   }
}

async function fetchProgramAll(name) {
   if(await fetchProgram(name)) return;
   if(await fetchProgram(name+".bin")) return;
   if(await fetchProgram(name+"/"+name)) return;
   if(await fetchProgram(name+"/"+name+".bin")) return;

   console.log(`cannot load "${name}": `, err);
}

async function fetchProgram(name)
{
   console.log(`wanting to load ${name}`);
   try
   {
      const response = await fetch(`software/${name}`);
      if(response.status === 404) return false;
      const bytes = new Uint8Array(await response.arrayBuffer());
      droppedFile(name, bytes);
      return true;
   }
   catch(err)
   {
      return false;      
   }
}

function rewind_tape() {   
   tapePtr = 0;
   tapeHighPtr = 0;
}

function stop_tape() {   
   tapePtr = tapeLen;   
}
