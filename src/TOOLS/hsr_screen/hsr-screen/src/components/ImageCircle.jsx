import { styled } from 'styled-components'
import blinkyGif from '../assets/blinky.gif'

const StyledCircle = styled.section`
  width: 60vh;
  color: #FFFFFF;
  box-sizing: border-box;
  height: 60vh;
  overflow: hidden;
  display: flex;
  flex-direction: column;
  border-radius: 500px;
  border: 20px solid #D63D16;
`

const StyledRoundedSquare = styled.section`
  color: #FFFFFF;
  box-sizing: border-box;
  height: 70vh;
  width: 52vw;
  overflow: hidden;
  display: flex;
  flex-direction: column;
  border-radius: 20px;
  border: 20px solid #D63D16;
`

const StyledImageBox = styled.div`
  display: flex;
  height: 100%;
  justify-content: center;
  align-items: center;
  flex-grow: 1;
  flex-shrink: 1;
`

const StyledImage = styled.img`
  height: 100%;
  width: auto;
`

const ImageCircle = ({ image }) => {
  return (
    <>
      
      { !image ?
        <StyledCircle>
          <StyledImageBox><StyledImage src={blinkyGif} /></StyledImageBox>
        </StyledCircle>
        :
        <StyledRoundedSquare>
          <StyledImageBox><StyledImage src={image} /></StyledImageBox>
        </StyledRoundedSquare>
      }
    </>
  )
}

export default ImageCircle