/*
 * ApproxMath.h
 * Copyright (C) 2022 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef APPROXMATH_H
#define APPROXMATH_H

float atan2_approx(float, float);
float sin_approx(float);
float cos_approx(float);
float approxHypotenuse(float, float);
float CosLat(float);
float InvCosLat(void);

int32_t iatan2_approx(int32_t ns, int32_t ew);
uint32_t iapproxHypotenuse(uint32_t dx, uint32_t dy);

#endif /* APPROXMATH_H */
