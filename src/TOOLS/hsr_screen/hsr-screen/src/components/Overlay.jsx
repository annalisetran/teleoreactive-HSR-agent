import { useEffect, useState, useRef } from 'react'

const Overlay = ({ message }) => {
  const [visible, setVisible] = useState(true)
  const [currentMessage, setCurrentMessage] = useState('')
  const timerRef = useRef(null)

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
      position: 'fixed',
      bottom: '20vh',
      left: '50%',
      transform: 'translateX(-50%)',
      background: '#000000b3',
      color: 'white',
      padding: '10px 20px',
      borderRadius: '8px',
      fontSize: '2em',
      zIndex: 9999,
      pointerEvents: 'none', // Let clicks pass through
    }}>
      {message.message}
    </div>
  )
}

export default Overlay
