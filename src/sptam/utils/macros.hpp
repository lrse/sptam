/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#pragma once

/** useful macros */
#define forn(i,n) for(size_t i=0;i<(n);i++)
#define fornr(i,n) for(size_t i=(n)-1;0<=i;i--)
#define forsn(i,s,n) for(size_t i=(s);i<(n);i++)
#define forsnr(i,s,n) for(size_t i=(n)-1;(s)<=i;i--)
#define forall(it,X) for(decltype((X).begin()) it=(X).begin();it!=(X).end();it++)
#define forallr(it,X) for(decltype((X).rbegin()) it=(X).rbegin();it!=(X).rend();it++)
