import { styled } from 'styled-components'
import OverlayHRI from './OverlayHRI'

const StyledBox = styled.section`
  width: 80%;
  color: #ffffff;
  box-sizing: border-box;
  height: 80%;
  overflow: hidden;
  display: flex;
  flex-direction: row;
  gap: 40px;
  font-size: 2.5rem;
`

const StyledIcon = styled.div`
  background: #000000b3;
  width: 8vw;
  height: 8vw;
  box-sizing: border-box;
  padding: 10px;
  border-radius: 20px;
`

const Textbox = ({ text, icon, color }) => {
  return (
    <>
      <StyledBox>
        <StyledIcon><img style={{width: '100%', height: 'auto'}} src={icon}/></StyledIcon>
        <div><OverlayHRI message={text} color={color}/></div>
      </StyledBox>
    </>
  )
}

export default Textbox