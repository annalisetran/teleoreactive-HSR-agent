import unswLogo from '../assets/unswLogo.png'

const footerStyle = {
  height: '8vh',
  width: '100%',
  display: 'flex',
  justifyContent: 'flex-end',
  boxSizing: 'border-box',
  padding: '10px',
  gap: '10px',
  alignItems: 'center',
  color: '#000000'
}

const Footer = () => {
  return (
    <footer style={footerStyle}>
      <p>robocup@Home Team</p>
      <img src={unswLogo} height={'100%'} alt={'Unsw logo'}/>
    </footer>
  )
}

export default Footer