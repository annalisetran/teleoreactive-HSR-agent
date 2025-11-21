import { useEffect, useState, useRef } from 'react'

const OverlayHRI = ({ message, color }) => {
  const [visible, setVisible] = useState(true)
  const [currentMessage, setCurrentMessage] = useState('')
  const timerRef = useRef(null)
  const [textColor, setTextColor] = useState('#ffffff')

  useEffect(() => {
    setTextColor(!color ? '#ffffff' : color)
  }, [color])

  useEffect(() => {
    if (message && message !== currentMessage) {
      setCurrentMessage(message.message)
      setVisible(true)
      
      if (timerRef.current) clearTimeout(timerRef.current);

      timerRef.current = setTimeout(() => {
        setVisible(false)
        setCurrentMessage('')
      }, 10000)
    }
  }, [message])
  if (!visible || !message || !message.message) {
    return (
      <></>
    )
  }
    

  return (
    <div style={{
      background: '#000000b3',
      color: textColor,
      padding: '10px 20px',
      borderRadius: '20px',
      fontSize: '2em'
    }}>
      {message.message}
    </div>
  )
}

export default OverlayHRI