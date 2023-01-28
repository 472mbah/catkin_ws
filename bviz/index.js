var http = require('http'); // 1 - Import Node.js core module

const produceHTMLBody = (robotPosition, blockers, path, visits, facingAngle=0, targetAngle=0, refreshRate=500) => {

    return `<!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta http-equiv="X-UA-Compatible" content="IE=edge">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Document</title>
        <meta http-equiv="refresh" content="10">
        <style>
            body {
                padding:0;
                margin:0
            }
        </style>
    </head>
    <body>
    
        <canvas></canvas>
        <script>
    
            const robotPosition = [${robotPosition[0]}, ${robotPosition[1]}]
            
            const blockers = {
                ${Object.keys(blockers).map(key=>"'"+key+"':null").join(',\n')}
            }
        
            const path = {
                ${path.map(([i, j])=> `'${i}:${j}'` + ":null").join(',\n')}
            }
        
            const visits = {
                ${Object.keys(visits).map(key=>"'"+key+"':null").join(',\n')}
            }
    
            const colours = {
                blockers:{background:'black', text:'white'},
                path:{background:'green', text:'black'},
                visits:{background:'orange', text:'black'},
                robot:{background:'red', text:'white'},
            }
    
            const n = 100;
    
            const canvas = document.querySelector('canvas');
            const ctx = canvas.getContext("2d");
            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
            const leftShift = 120;
            const topShift = 0;
            const BOXDIMENSIONS = window.innerHeight / n
            const foundDimensions = window.innerHeight
            
            ctx.fillRect( leftShift, topShift, window.innerHeight, window.innerHeight )
            
            const midN = Math.floor(n/2)
            const minI = robotPosition[0] - midN
            const minJ = robotPosition[1] - midN
    
            let exactMidPoint;
    
            for (let i = 0; i < n; i++) {
                for (let j = 0; j < n; j++) {
                    let shifteI = minI + i
                    let shifteJ = minJ + j
                    let id = shifteI+":"+shifteJ
                    let bcolor;
                    let tcolor;

                    if (shifteI==robotPosition[0] && shifteJ==robotPosition[1]) {
                        bcolor = colours.robot.background;
                        tcolor = colours.robot.text;
                        exactMidPoint = {
                                x:i*BOXDIMENSIONS + leftShift + BOXDIMENSIONS/2, 
                                y:j*BOXDIMENSIONS + topShift + BOXDIMENSIONS/2
                            }
                    }
                    else if (blockers.hasOwnProperty(id)) {
                        bcolor = colours.blockers.background;
                        tcolor = colours.blockers.text;
                    }
                    else if (visits.hasOwnProperty(id)) {
                        bcolor = colours.visits.background;
                        tcolor = colours.visits.text;
                    }
                    else if (path.hasOwnProperty(id)) {
                        bcolor = colours.path.background;
                        tcolor = colours.path.text;
                    }else if (shifteI===0 || shifteJ===0) {
        
                        bcolor = "#ddd";
                    }
                    else {
                        bcolor = 'white'
                        tcolor = 'black'
                    }
                    // ctx.fillStyle = tcolor;
                    // ctx.fillText(id, i*BOXDIMENSIONS + leftShift, j*BOXDIMENSIONS + topShift);
                    ctx.fillStyle = bcolor;
                    ctx.fillRect( i*BOXDIMENSIONS + leftShift, j*BOXDIMENSIONS + topShift, BOXDIMENSIONS, BOXDIMENSIONS )
                }   
            }
    
    
            let trueIRange = n*2
            ctx.fillStyle = "#000";
            ctx.font = "18px Arial";
            canvas.onmousemove = function(e) {
                let x = e.clientX
                let y = e.clientY
                if (x < leftShift) return;
                if (x > (leftShift + foundDimensions)) return;
                if (y < topShift) return;
                if (y > (topShift + foundDimensions)) return;
                let j = Math.floor((x-leftShift)/BOXDIMENSIONS)
                let i = Math.floor((y-topShift)/BOXDIMENSIONS)
                ctx.clearRect(0, 0, leftShift, 40)
                ctx.fillText((minI+i)+","+(minJ+j), 30, 30);
            }
    
            if (exactMidPoint) {
                
                const robotAngleFacing = ${facingAngle}
                const targetAngle = ${targetAngle}
                let vx = Math.cos(robotAngleFacing)
                let vy = Math.sin(robotAngleFacing)
    
                let tx = Math.cos(targetAngle)
                let ty = Math.sin(targetAngle)
                ctx.strokeStyle = "red"
                ctx.beginPath();
                ctx.moveTo(exactMidPoint.x, exactMidPoint.y);
                let newX = ((vx*100)+exactMidPoint.x)
                let newY = (vy*100)+exactMidPoint.y
                ctx.lineTo(newX, newY);
                ctx.stroke();
                ctx.closePath();
                ctx.fillStyle = "red";
                ctx.fillRect( newX-BOXDIMENSIONS/4, newY-BOXDIMENSIONS/4, BOXDIMENSIONS/2, BOXDIMENSIONS/2 )
            
                ctx.strokeStyle = "green"
                ctx.beginPath();
                ctx.moveTo(exactMidPoint.x, exactMidPoint.y);
                let newXt = (tx*100)+exactMidPoint.x
                let newYt = (ty*100)+exactMidPoint.y
                ctx.lineTo(newXt, newYt);
                ctx.stroke();
                ctx.closePath();
                ctx.fillStyle = "green";
                ctx.fillRect( newXt-BOXDIMENSIONS/4, newYt-BOXDIMENSIONS/4, BOXDIMENSIONS/2, BOXDIMENSIONS/2 )
    
            }
            
        
        </script>
    </body>
    
    </html>`
    
    
   
}

let blockers = {'2:0':null, '3:20':null, '13:5':null, '19:18':null}
let visits = {'12:0':null, '2:20':null, '19:5':null, '27:18':null}
let path = [ [1,2], [1,3], [1,4], [1,5] ]
let robotPosition = [1,6]
let targetAngle = 0
let facingAngle = 0

const setValues = (body) => {
    blockers = body.blockers
    visits = body.visits
    path = body.path
    facingAngle = body.facingAngle
    targetAngle = body.targetAngle
    robotPosition = body.robotPosition
}

var server = http.createServer(function (req, res, body) {   // 2 - creating server

    if (req.method=="POST") {
        
        let body = [];
        req.on('data', (chunk) => {
        body.push(chunk);
        }).on('end', () => {
            body = Buffer.concat(body).toString();
            setValues(JSON.parse(body));
        });

        res.writeHead(200, { 'Content-Type': 'application/json' }); 
        res.write(JSON.stringify({"message":'okay!'}));
        res.end();

        return;
    }

    res.writeHead(200, { 'Content-Type': 'text/html' }); 
    
    res.write(produceHTMLBody(robotPosition, blockers, path, visits, facingAngle, targetAngle));
    res.end();

});

server.listen(9000); //3 - listen for any incoming requests

console.log('Node.js web server at port 9000 is running..')