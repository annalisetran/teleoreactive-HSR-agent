import styled from 'styled-components';
import blinkyIcon from '../assets/blinkyIcon.png';
import Battery from './Battery';
import React from 'react';

const navBarStyle = {
  backgroundColor: '#ffe600',
  width: '100%',
  height: '15vh',
  color: '#000000',
  padding: '10px',
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

const StyledImage = styled.img`
  height: 14vh;
`

const NavBar = ({charge}) => {
  return <>
    <header style={navBarStyle}>
      <HeaderComponent>
        <StyledImage src={blinkyIcon} alt={'robot icon'}/>
      </HeaderComponent>
      <HeaderComponent>
        <div style={{fontSize: '3em', fontWeight: '700'}}>Blinky the HSR</div>
      </HeaderComponent>
      <Battery charge={charge}/>
    </header>
  </>
}

export default NavBar