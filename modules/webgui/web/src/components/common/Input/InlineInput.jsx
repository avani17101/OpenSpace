import React from 'react';
import PropTypes from 'prop-types';
import AutosizeInput from 'react-input-autosize';
import Input from './Input';
import styles from './InlineInput.scss';

const InlineInput = props => (
  <AutosizeInput
    {...props}
    value={props.value}
    onChange={this.onChange}
    className={`${styles.input} ${props.className}`}
  />
);

InlineInput.propTypes = {
  className: PropTypes.string,
  id: PropTypes.string,
  onChange: PropTypes.func,
  type: PropTypes.string,
  value: PropTypes.oneOfType([PropTypes.string, PropTypes.number]),
};

InlineInput.defaultProps = {
  className: '',
  id: `input-${Input.nextId}`,
  onChange: () => {},
  type: 'text',
  value: '',
};

export default InlineInput;
