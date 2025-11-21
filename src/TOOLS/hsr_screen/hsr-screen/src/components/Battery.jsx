import styled from 'styled-components';
import batteryFull from '../assets/batteryFull.svg';
import batteryEmpty from '../assets/batteryEmpty.svg';
import batteryLow from '../assets/batteryLow.svg';
import batteryMid from '../assets/batteryMid.svg';
import React from 'react';

const StyledBattery = styled.img`
  height: 7vh;
  width: auto;
`

const Battery = ({charge}) => {
  const [battery, setBattery] = React.useState(batteryEmpty)

  React.useEffect(() => {
    if (charge < 20) {
      setBattery(batteryEmpty)
    } else if (charge < 45) {
      setBattery(batteryLow)
    } else if (charge < 70) {
      setBattery(batteryMid)
    } else {
      setBattery(batteryFull)
    }
  }, [charge])

  return <>
      <div style={{display: 'flex', alignItems: 'center'}}>
        <div style={{ fontWeight: 'bold', fontSize: '1.2em'}}>{charge}</div>
          <StyledBattery src={battery} alt={'Battery'}/>
      </div>
  </>
}

export default Battery