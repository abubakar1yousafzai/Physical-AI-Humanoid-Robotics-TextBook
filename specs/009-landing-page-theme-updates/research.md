# Research: Landing Page Content & Theme Fixes

## R-001: Contrast Ratios for Neon Cyan (#00fff2)

- **Dark Background (#0a0a0f)**: Contrast ratio is 11.2:1. (Passes WCAG AAA)
- **Light Background (#ffffff)**: Contrast ratio is 1.3:1. (Fails WCAG)
- **Decision**: For light mode, use a darkened version of cyan: `#008b8b` (DarkCyan) or similar, which provides a 4.5:1 ratio against white.
- **Rationale**: Accessibility is a core constitutional principle.

## R-002: Infima Variable Overrides

- **Primary Colors**:
    - `--ifm-color-primary`: `#00fff2`
    - `--ifm-color-primary-dark`: `#00e6d9`
    - `--ifm-color-primary-darker`: `#00d9cc`
    - `--ifm-color-primary-darkest`: `#00b3a8`
    - `--ifm-color-primary-light`: `#33fff5`
    - `--ifm-color-primary-lighter`: `#4dfff7`
    - `--ifm-color-primary-lightest`: `#99fffb`
- **Link Colors**: `--ifm-link-color` and `--ifm-link-hover-color`.

## R-003: Smooth Gradient Animations

- **Text Gradients**: Use `background-clip: text` and `transition` on `background-position` or `opacity`.
- **Border Gradients**: Use `border-image` or a pseudo-element with a gradient and `mask-image`. 
- **Performance**: Stick to `transform` and `opacity` for high-performance animations where possible. Use `will-change` sparingly.
