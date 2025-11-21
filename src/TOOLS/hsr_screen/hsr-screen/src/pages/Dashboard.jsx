import ImageCircle from "../components/ImageCircle"
import NavBar from '../components/NarBar'

const sectionStyle = {
  padding: '20px',
  boxSizing: 'border-box',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  flexDirection: 'row',
  height: '77vh',
  gap: '20px'
}

const Dashboard = ({ charge }) => {
  return <>
    <NavBar charge={charge}/>
    <section style={sectionStyle}>
      <ImageCircle></ImageCircle>
    </section>
  </>
}

export default Dashboard