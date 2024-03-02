Andrew51 Andrew Chang

### Problem 1

Prove that the impulse is the identity of
convolution.

Given a function (or array) $f$, show that $f ∗ δ = f$.

First, take $f$ and convolve with $δ$

$$
(f*\delta)(t) = \int_{-\infin}^{\infin} f(\tau)*\delta(t-\tau) d\tau
$$

We know that $\delta(t-\tau)$ is 1 at $t = \tau$ but 0 at all
other t, so we can simplify the integral to

$$
(f*\delta)(t) = f(t)\int_{-\infin}^{\infin}\delta(t-\tau)d\tau
$$

$$
(f*\delta)(t) = f(t)
$$

### Problem 2

Prove the differentiation
property of convolution. That is, given functions f and g, show that d
dx (f ∗ g) = df
dx ∗ g. Again, you may
consider the functions to be one dimensional. Please show all steps and justify where necessary. Explain
the signifinace of this property in two sentences as it pertains to filtering.

By the definition of convolution,

$$
(f*g)(t) = \int_{-\infin}^{\infin} f(\tau)*g(t-\tau) d\tau
$$

Then, we can take the derivative

$$
\frac{d}{dx}(f*g) = \frac{d}{dx}\int_{-\infin}^{\infin} f(τ)g(x - τ) dτ
$$

$$
 =\int_{-\infin}^{\infin}  f(τ) \frac{d}{dx}[g(x - τ)] dτ
$$

$$
 =f*\frac{dg}{dx}
$$

Since we can do $f*g = g*f$, we can see that

$$
\frac{d}{dx}(f*g) = \frac{df}{dx}*g
$$

### Problem 3

Compute the partial derivative with respect to y of
the two dimensional Gaussian Gσ(x, y).

The formula for Gσ(x, y) can be represented as

$$
G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2+y^2}{\sigma^2}}
$$

$$
\frac{d}{dy}G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} \frac{d}{dy}e^{-\frac{x^2+y^2}{\sigma^2}}
$$

$$
\frac{d}{dy}G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2+y^2}{\sigma^2}}\frac{-2y}{\sigma^2}
$$

$$
\frac{d}{dy}G_\sigma(x,y) = G_\sigma(x,y)\frac{-2y}{\sigma^2}
$$

### Problem 4

The median filter is non-linear, and to show this we can try to prove that its linear
by using the definition of a linear function with filter f and two example
matrices g1 and g2.

$$
f*(g_1+g_2) = f*g_1 + f*g_2
$$

Given two example matrices

$$
g_1 =
\begin{bmatrix} 4,7,2 \\\ 3,8,1 \\\ 6,5,9  \end{bmatrix}
g_2 =
\begin{bmatrix} 9,1,5 \\\ 2,6,3 \\\ 8,7,4   \end{bmatrix}
$$

Taking the sum, we get

$$
g_1+g_2 =
\begin{bmatrix} 13,8,7 \\\ 5,14,4 \\\ 14,12,13   \end{bmatrix}
$$

Applying the filter, we get

$$

\begin{bmatrix} 12,12,12 \\\ 12,12,12 \\\ 12,12,12   \end{bmatrix}
$$

For the individual ones, we get

$$
f*g_1 = \begin{bmatrix} 5,5,5 \\\ 5,5,5 \\\ 5,5,5   \end{bmatrix}
f*g_2 = \begin{bmatrix} 5,5,5 \\\ 5,5,5 \\\ 5,5,5   \end{bmatrix}
$$

$$
f*g_1+f*g_2 = \begin{bmatrix} 10,10,10 \\\ 10,10,10 \\\ 10,10,10   \end{bmatrix}
$$

So, we proved that the median filter is non-linear

### Problem 5

Which filter is better for filtering out
salt-and-pepper noise? Which filter is better for filtering out Gaussian (White) noise? Attach your result
images in the report and explain why you think the specific filters work better for certain noises? (You will
need to do source devel/setup.bash at this step. Refer to later sections for more detail)

Here are the results for applying the Gaussian filter:

**Gaussian filter w/ Gaussian**
![alt text](Gaussian:Gaussian.png "Title")

**Gaussian filter w/ Median**
![alt text](Gaussian:Median.png "Title")

As you can see, the Gaussian filter seems to be slightly better for the
gaussian noise because it smoothes it out by taking the average.

Here are the results for applying the Median filter:

**Salt-and-Pepper filter w/ Median**
![alt text](sp:Gaussian.png "Title")
**Salt-and-Pepper filter w/ Median**
![alt text](sp:median.png "Title")

For the salt and peper noise, the noise takes form in pixels with extreme
values that are either realy dark or really light. The median filter is
perfect for this since it takes the median value and ignores the really
extreme ends, which results in a really clear photo after filtering.
