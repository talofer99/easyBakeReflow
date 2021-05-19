const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/chart.js@2.8.0"></script>

</head>
<body>
  <h2>ESP Web Server</h2>
  <canvas id="myChart"></canvas>
  <button id="addData">add</button>
  <script>


// ********************************************************
//  WEB SOCKET
// ********************************************************
var connection = new WebSocket('ws://'+location.hostname+'/ws', ['arduino']);
// on open
connection.onopen = function () {  }; 
// on error
connection.onerror = function (error) {    console.log('WebSocket Error ', error);};

// on message 
connection.onmessage = function (e){
	console.log(e.data);
	//parse json
    addToGraph(e.data)
	//jsonObj = JSON.parse(e.data);
	//console.log('Server: ', jsonObj);
	
};

var ctx = document.getElementById('myChart').getContext('2d');

var config = {
    // The type of chart we want to create
    type: 'line',

    // The data for our dataset
    data: {
        labels: [],
        datasets: [{
            label: 'Temp',
            backgroundColor: 'rgb(255, 99, 132)',
            borderColor: 'rgb(255, 99, 132)',
			fill: false,
            data: []
        }]
		
    },

    // Configuration options go here
    options: {
		responsive: true,
         
	}
};

window.myLine = new Chart(ctx, config);

var i = 1;

var addToGraph = function(temp)
{
    //alert(temp);
    /*if (config.data.datasets.length > 0) {
				var month = MONTHS[config.data.labels.length % MONTHS.length];
				config.data.labels.push(month);

				config.data.datasets.forEach(function(dataset) {
					dataset.data.push(randomScalingFactor());
				});

				window.myLine.update();
			}*/
			config.data.labels.push(i);
			config.data.datasets[0].data.push(temp);
			//console.log(config.data.datasets[0].data);
			//alert("push");
			i++;
			window.myLine.update();
}

document.getElementById('addData').addEventListener('click', function() {
			/*if (config.data.datasets.length > 0) {
				var month = MONTHS[config.data.labels.length % MONTHS.length];
				config.data.labels.push(month);

				config.data.datasets.forEach(function(dataset) {
					dataset.data.push(randomScalingFactor());
				});

				window.myLine.update();
			}*/
			config.data.labels.push(i);
			config.data.datasets[0].data.push(i);
			//console.log(config.data.datasets[0].data);
			//alert("push");
			i++;
			window.myLine.update();
		});

</script>
</body>
</html>
)rawliteral";