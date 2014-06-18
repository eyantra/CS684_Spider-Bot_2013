



<html>
<head>
<link rel="stylesheet" type="text/css" href="style.css" />

<!--function for IP camera pan tilt control-->
<script type="text/javascript">

var user="usr1";
var pwd="test";

function decoder_control_2(command)
{ 
    action_zone.location='http://greenhouse.cse.iitb.ac.in:8090/decoder_control.cgi?command='+command+'&usr='+user+'&pwd='+pwd;
}
</script>


</head>
<body>

<!--navigation bar-->
<div id="navbar">
	<div id="holder">
<ul>
<li><a href="#" id="onlink">Home</a></li>
<li><a href="auto2.php">Auto</a></li>
<li><a href="..\google_surface_plot\temp_prof.php">TempProfile</a></li>

</ul>

</div>
</div>
<div id="wrapper">



<center><h1>Spyder GUI control</h1></center>
<!--camera image-->
<div id="camera">
<img src="http://greenhouse.cse.iitb.ac.in:8090/videostream.cgi?user=user1&pwd=test" width="700" height="450" id="pic">
</div>

<div id="map">
<canvas id="myCanvas" width="289" height="100" ></canvas>
    <script>
      var canvas = document.getElementById('myCanvas');
      var context = canvas.getContext('2d');
      var centerX = canvas.width / 2;
      var centerY = canvas.height / 2;
      var radius = 10;
      var imageObj = new Image();
     
      context.beginPath();
      context.arc(centerX, centerY, radius, 0, 2 * Math.PI, false);
      context.fillStyle = 'red';
      context.fill();
      context.lineWidth = 5;
      context.strokeStyle = '#003300';
      context.stroke();
    </script>
</div>


<!--buttons for platform motor control-->
<div id="controls">
<form method="post" action="<?php echo $_SERVER['PHP_SELF'];?>">

<div id="left">
<button type="submit"  value="Left" name="rcmd" style="border: 0; background: transparent" title="left">
    <img src="button_left.png" width="90" height="90" alt="submit" />
</button>
</div>

<div id="right">
<button type="submit"  value="Right" name="rcmd" style="border: 0; background: transparent" title="right">
    <img src="button_right.png" width="90" height="90" alt="submit" />
</button>
</div>

<div id="stop">
<button type="submit"  value="Stop" name="rcmd" style="border: 0; background: transparent" title="stop">
    <img src="button_stop.png" width="90" height="90" alt="submit" />
</button>
</div>

<div id="forward">
<button type="submit"  value="Forward" name="rcmd" style="border: 0; background: transparent" title="forward">
    <img src="button_forward.png" width="90" height="90" alt="submit" />
</button>
</div>

<div id="backward">
<button type="submit"  value="Backward" name="rcmd" style="border: 0; background: transparent" title="backward">
    <img src="button_backward.png" width="90" height="90" alt="submit" />
</button>

</div>
<div id="up">
<input type="submit" value="Up" name="rcmd" title="up">
</div>

</form>

</div>

<!--buttons for camera pan tilt control-->
<div id="camera_control">
<iframe name="action_zone" style="DISPLAY: none" width="0" height="0">
</iframe>

<button type="button" onTouchStart="decoder_control_2(0)" onTouchEnd="decoder_control_2(1)" onMouseDown="decoder_control_2(0)" onMouseUp="decoder_control_2(1)" style="border: 0; background: transparent" title="backward">
<img src="button_forward.png" width="90" height="90" alt="submit" />
</button>
<button type="button" onTouchStart="decoder_control_2(2)" onTouchEnd="decoder_control_2(3)" onMouseDown="decoder_control_2(2)" onMouseUp="decoder_control_2(3)"style="border: 0; background: transparent" title="backward">
<img src="button_backward.png" width="90" height="90" alt="submit" />
</button>
<button type="button" onTouchStart="decoder_control_2(4)" onTouchEnd="decoder_control_2(5)" onMouseDown="decoder_control_2(4)" onMouseUp="decoder_control_2(5)"style="border: 0; background: transparent" title="backward">
<img src="button_right.png" width="90" height="90" alt="submit" />
</button>
<button type="button" onTouchStart="decoder_control_2(6)" onTouchEnd="decoder_control_2(7)" onMouseDown="decoder_control_2(6)" onMouseUp="decoder_control_2(7)"style="border: 0; background: transparent" title="backward">
<img src="button_left.png" width="90" height="90" alt="submit" />
</button>

</div>

</div>

<!--php code for serial communication and sending commands on button press
<?php


$verz="1.0";
$comPort = "/dev/ttyACM1"; /*change to correct com port */

if (isset($_POST["rcmd"])) {
$rcmd = $_POST["rcmd"];
switch ($rcmd) {
     case 'Stop':
		echo "Stop";
       $fp =fopen($comPort, "w");
  fwrite($fp, 's'); /* this is the char that it will write */
  fclose($fp);
  break;
     case 'Forward':
		echo "Forward";
        $fp =fopen($comPort, "w");
  fwrite($fp, 'f'); /* this is the char that it will write */
  fclose($fp);
  break;
  case 'Backward':
		echo "Backward";
        $fp =fopen($comPort, "w");
  fwrite($fp, 'b'); /* this is the char that it will write */
  fclose($fp);
  break;
  case 'Up':
		echo "up";
        $fp =fopen($comPort, "w");
  fwrite($fp, 'u'); /* this is the char that it will write */
  fclose($fp);
  break;
case 'Right':
		echo "right";
        $fp =fopen($comPort, "w");
  fwrite($fp, 'r'); /* this is the char that it will write */
  fclose($fp);
  break;
case 'Left':
		echo "left";
        $fp =fopen($comPort, "w");
  fwrite($fp, 'l'); /* this is the char that it will write */
  fclose($fp);
  break;
default:
  die('Select the correct options');
}

}
?>
 



</body>
</html>
