
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("TT").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/TT", true);
  xhttp.send();
}, 10000 ) ;


setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("TH").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/TH", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("THI").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/THI", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("GRX").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/GRX", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("GRY").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/GRY", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("GRZ").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/GRZ", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("GT").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/GT", true);
  xhttp.send();
}, 10000 ) ;




setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("AX").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/AX", true);
  xhttp.send();
}, 10000 ) ;


setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("AY").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/AY", true);
  xhttp.send();
}, 10000 ) ;



setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("AZ").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/AZ", true);
  xhttp.send();
}, 10000 ) ;


function StartM() {
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", "/startM", true);
  xhttp.send();
}


function StopM() {
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", "/stopM", true);
  xhttp.send();
}

