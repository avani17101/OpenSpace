/* eslint import/no-extraneous-dependencies: 0 */
import React from 'react';
import { storiesOf } from '@storybook/react';
import { action } from '@storybook/addon-actions';
import NumericInput from './NumericInput';

/* globals module */

storiesOf('Input/NumericInput', module)
  .addDecorator(story => (
    <div style={{ background: '#252525', padding: '20px' }}>
      { story() }
    </div>
  ))
  .add('default', () => (<NumericInput placeholder="NumericInput" value={52} />))
  .add('no input', () => (<NumericInput placeholder="NumericInput" />))
  .add('unwide', () => (<NumericInput placeholder="NumericInput" wide={false} />))
  .add('disabled', () => (<NumericInput placeholder="NumericInput" value={33.3} disabled />))
  .add('inputOnly ', () => (<NumericInput placeholder="NumericInput" value={33.3} inputOnly />))
  .add('noHoverHint ', () => (<NumericInput placeholder="NumericInput" value={33.3} noHoverHint />))
  .add('disableInput ', () => (<NumericInput placeholder="NumericInput" value={33.3} disableInput />))
  .add('with callback', () =>
    (<NumericInput placeholder="Input" value="change me" onChange={action('input')} />));
