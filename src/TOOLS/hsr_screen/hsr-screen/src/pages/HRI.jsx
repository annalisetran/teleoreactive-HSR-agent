import ImageCircle from "../components/ImageCircle"
import Overlay from "../components/Overlay"
import NavBarObjects from "../components/NavBarObjects"
import Textbox from "../components/Textbox"
import styled from "styled-components"
import robotIcon from "../assets/robot.svg"
import personIcon from "../assets/person.svg"

const StyledSection = styled.section`
  padding: 20px;
  box-sizing: border-box;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 77vh;
  gap: 20px;
`

const HRI = ({ output, input, charge }) => {
  return <>
    <NavBarObjects charge={charge}/>
    <StyledSection>
      <Textbox icon={robotIcon} text={output}></Textbox>
      <Textbox icon={personIcon} text={input} color={'#ffe600'}></Textbox>
    </StyledSection>
  </>
}

export default HRI