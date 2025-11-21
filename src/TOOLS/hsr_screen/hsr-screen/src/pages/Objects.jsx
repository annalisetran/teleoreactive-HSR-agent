import ImageCircle from "../components/ImageCircle"
import NavBarObjects from "../components/NavBarObjects"
import Overlay from "../components/Overlay"

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

const Objects = ({ output, objectClass, image, charge }) => {
  return <>
    <NavBarObjects charge={charge} objectClass={objectClass}/>
    <section style={sectionStyle}>
      <ImageCircle image={image}></ImageCircle>
    </section>
    <Overlay message={output}/>
  </>
}

export default Objects