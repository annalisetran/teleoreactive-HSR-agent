import { Routes, Route, useLocation } from 'react-router-dom'
import Dashboard from './pages/Dashboard'
import Objects from './pages/Objects'
import HRI from './pages/HRI'
import { useEffect, useState } from 'react'
import ROSLIB from 'roslib'
import Footer from './components/Footer'
import './index.css';

// Colour Palette: https://paletton.com/#uid=13L0u0kj4Gp8EVpenOon8yPsbuk

function Router() {
  const [output, setOutput] = useState({})
  const [input, setInput] = useState('')
  const [charge, setCharge] = useState(0)
  const [image, setImage] = useState('')
  const [objectClass, setObjectClass] = useState('')
  const [dashboard, setDashboard] = useState(true)
  const [path, setPath] = useState(useLocation().pathname)

  useEffect(() => {
    
    const ros = new ROSLIB.Ros({
      url: 'ws://hsrb.local:9090'
      // url: 'ws://localhost:9090' // Use this for local testing
    });

    ros.on('connection', () => console.log("Connected to ROS"));
    ros.on('error', (error) => console.error('ROS Error: ', error))
    ros.on('close', () => console.log('Connection closed'))

    const outputListener = new ROSLIB.Topic({
      ros,
      name: "/tts/phrase",
      messageType: "std_msgs/String"
    });

    const inputListener = new ROSLIB.Topic({
      ros,
      name: "/speech_recognition/final_result",
      messageType: "std_msgs/String"
    });

    const chargeListener = new ROSLIB.Topic({
      ros,
      name: "/hsrb/battery_state",
      messageType: "tmc_msgs/BatteryState"
    });

    const imageListener = new ROSLIB.Topic({
      ros,
      name: "/unsw_vision/object_frame/image",
      messageType: "std_msgs/String"
    });

    const classListener = new ROSLIB.Topic({
      ros,
      name: "/unsw_vision/object_frame/object_class",
      messageType: "std_msgs/String"
    });


    outputListener.subscribe((msg) => {
      setOutput({message: msg.data, time: Date.now()})
      setDashboard(false)
    })

    inputListener.subscribe((msg) => {
      console.log('Received input message: ', msg)
      setInput({message: msg.data, time: Date.now()})
      if (path === '/hri') setDashboard(false)
    })

    chargeListener.subscribe((msg) => {
      console.log('received battery message: ', msg)
      setCharge(Math.round(msg.power))
    })

    imageListener.subscribe((msg) => {
      console.log('received object image')
      setImage(msg.data)
      if (path !== '/hri') setDashboard(false)
    })

    classListener.subscribe((msg) => {
      console.log('received object image')
      setObjectClass(msg.data)
      if (path !== '/hri') setDashboard(false)
    })


    return () => {
      outputListener.unsubscribe()
      inputListener.unsubscribe()
      chargeListener.unsubscribe()
      imageListener.unsubscribe()
      classListener.unsubscribe()
      ros.close()
    }
  }, [])
  return (
    <>
      { dashboard ? (
        <Dashboard output={output} input={input} charge={charge} />
      ) : (
        <Routes>
          <Route path='/' element={<Objects output={output} objectClass={objectClass} image={image} charge={charge}/>}/>
          <Route path='/hri' element={<HRI output={output} input={input} charge={charge}/>}/>
        </Routes>
      )
      }
      
      <Footer/>
    </>
  )
}

export default Router