import styled from 'styled-components';
import blinkyIcon from '../assets/blinkyIcon.png';
import React from 'react';
import Battery from './Battery';

const navBarStyle = {
  width: '100%',
  height: '15vh',
  color: '#000000',
  paddingTop: '60px',
  boxSizing: 'border-box',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  letterSpacing: '3px',
  gap: '60px',
}

const HeaderComponent = styled.div`
  display: flex;
  align-items: center;
`

const HeaderComponentColumn = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
`

const StyledImage = styled.img`
  height: 14vh;
`

const NavBarObjects = ({charge, objectClass}) => {
  return <>
    <header style={navBarStyle}>
      <HeaderComponent>
        <StyledImage src={blinkyIcon} alt={'robot icon'}/>
      </HeaderComponent>
      <HeaderComponentColumn>
        { objectClass && (
          <>
            <div style={{fontSize: '3em', fontWeight: '700'}}>Object Detected:</div>
            <div style={{fontSize: '3em', fontWeight: '500'}}>{objectClass}</div>
          </>
        )}
      </HeaderComponentColumn>
      <Battery charge={charge}/>
    </header>
  </>
}

export default NavBarObjects