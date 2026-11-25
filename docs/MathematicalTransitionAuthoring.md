# Mathematical Transitions within Regions

Select a Region in the Region List and an engineering profile in the Transition Editor. Open **Transitions**, then choose a station interval from the list or click its curve in the graph. The selected span is highlighted. The **Keyframes** controls and graph handles continue to edit the shared keyframes. Graph navigation and Region Properties are available throughout.

Every profile has at least one span covering part or all of its Region. **Prepend** and **Append** split the selected span at its midpoint and select a new Linear span on the chosen side. The shared key takes its initial value from the old curve at the split. A nonlinear span keeps its function on the surviving interval and evaluates it over that shorter interval. **Delete** merges the selected span into the span on its right, or into the one on its left when it is the last span. The remaining span keeps its function and is evaluated over the longer interval.

Set a function, direction, length, and endpoint values, then choose **Apply**. Changing a shared endpoint affects both adjacent spans. A length edit moves the end key when it is inside the Region and the start key when the span ends at the Region boundary. Region Properties resizes a whole Region and scales its interior key stations proportionally. Downstream Regions move with the join. Edit a shared Region boundary from the preceding Region's last Transition. Reconstruction updates the following Region's linked inputs. Splitting, applying, merging, and dragging use the same committed edit history. The graph can preview an interior keyframe drag, and the 3D track updates when the edit commits.

The workflow follows the FVD++ pattern of selecting an authored profile and editing the type, length, and values of one Transition at a time. QUANTUM DYNAMIX's Region extents define a fixed coverage interval for each profile. [FVD++ documentation, §§ 4.1–4.5](https://lucasbosch.de/nolimits-tools/FVD%2B%2B/older_versions/fvd%2B%2B_0.5_documentation.pdf)

## Function conventions

Let $u=(s-s_0)/(s_1-s_0)$ be the fraction through a span. A shaped profile evaluates as $y(s)=y_0+(y_1-y_0)E(u)$, where $E(0)=0$ and $E(1)=1$. These shapes are monotone between the chosen endpoints, with decreasing values if $y_1<y_0$. For a fixed shape and value change, shortening the span makes the first derivative steeper in inverse proportion to length. The second and third derivatives scale with the inverse square and cube of length.

| Function | Base **In** shape and endpoint behavior |
| --- | --- |
| Constant | One value throughout. Both endpoints must be equal. All derivatives are zero. |
| Linear | $E(u)=u$. Unit slope at both ends in normalized coordinates. |
| Quadratic / Cubic / Quartic / Quintic | $E(u)=u^n$ for $n=2,3,4,5$. The starting derivatives through order $n-1$ vanish. The ending slope is $n$. |
| Sine / Cosine | Both library presets use $E(u)=1-\cos(\pi u/2)$. The starting slope is zero and the ending slope is $\pi/2$. |
| Plateau | Monotone change, hold, and change to the ending value with seventh-degree shoulders. The hold starts and ends at fractions $a,b$ of the base span and sits at fraction $h$ of the value change. Use $0<a<b<1$ and $0<h<1$. The shoulders match the flat portions through the third derivative. |

**Out** reflects the base function as $1-E(1-u)$. **InOut** uses $E(2u)/2$ on the first half and $1-E(2-2u)/2$ on the second. InOut Plateau therefore mirrors the hold with the second hold at the complementary value. InOut powers are continuous through the first derivative at the midpoint, where the second derivative changes sign. The shared Sine and Cosine presets give a smooth half-cosine in InOut. Plateau is smooth through the third derivative at its shoulder joins and midpoint. Across adjacent authored spans only the value is required to match. Higher-order continuity depends on the selected shapes, endpoint changes, and lengths on both sides.

The application's function list uses the existing normalized library easings. The library accepts caller-supplied C# delegates through `EasingFunction.Custom`. Project files support the listed presets. [Native project format](NativeProjectFormat.md)

## Geometric Regions

Choose **Geometric Region** in the existing Append Region or Prepend Region form. Edit **Pitch Change (°/m)**, **Yaw Change (°/m)**, or **Banking (rad)**. Pitch and yaw are ground-relative angular rates per metre of heartline. Positive pitch rotates the tangent toward increasing unwrapped pitch, starting toward +Z from level. Positive yaw rotates carried heading from +X toward +Y about world +Z. Banking is the existing absolute, unwrapped roll angle about the forward tangent, measured in radians.

For carried heading $\theta$ and unwrapped pitch $\phi$, the tangent is $T=(\cos\phi\cos\theta,\cos\phi\sin\theta,\sin\phi)$. The solver integrates yaw change to obtain $\theta$ and evaluates both authored rates at the Runge–Kutta stations. The world rotation vector per metre is $\theta'(0,0,1)+\phi'(\sin\theta,-\cos\theta,0)$. Its cross product with $T$ changes the tangent. The solver drops the rotation component parallel to $T$ when transporting the unbanked frame, integrates the tangent to get world position, then applies the separate banking angle. At vertical, yaw still changes carried heading and has zero instantaneous effect on the tangent.

The readout uses reconstructed geometry at a selected keyframe or the midpoint of a selected span. It shows curvature magnitude, world elevation and compass heading, their rates where defined, and calculated normal and lateral specific force at the prescribed speed. The compass heading shown is the direction of the horizontal tangent projection, so it can flip by 180° after passing through vertical even as the authored heading carries through. At a vertical tangent the readout omits the compass angle and its associated rates.

An adjoining Region is initialized from the shared position, tangent, bank, and 3D curvature. Where modes change, reconstruction recomputes the linked opening inputs after upstream edits. A Geometric Region converts the incoming world curvature into pitch change and yaw change. At a vertical tangent, pitch change can represent curvature in the plane selected by the carried heading. An out-of-plane request is rejected with a diagnostic. An incompatible Constant span at a linked opening key also rejects the edit if rebinding would require unequal endpoints. Rejected edits preserve committed geometry and Undo/Redo history. The workflow uses prescribed constant speed and the existing wireframe cross section.
