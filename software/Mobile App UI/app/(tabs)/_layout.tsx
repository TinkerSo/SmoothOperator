import { Tabs } from 'expo-router';
import React from 'react';
import { Platform } from 'react-native';

import { Colors } from '@/constants/Colors';
import { useColorScheme } from '@/hooks/useColorScheme';

export default function TabLayout() {
  const colorScheme = useColorScheme();

  return (
    <Tabs
      screenOptions={{
        // Hiding the tab bar by setting display to 'none'
        tabBarStyle: { display: 'none' },
        headerShown: false,
        // The following options are optional if you're not using them:
        tabBarActiveTintColor: Colors[colorScheme ?? 'light'].tint,
        // If you have a custom tab button or background, you can leave them here
        // tabBarButton: HapticTab,
        // tabBarBackground: TabBarBackground,
        // For iOS-specific styling, you can remove or keep this based on your needs:
        // tabBarStyle: Platform.select({
        //   ios: {
        //     position: 'absolute',
        //     display: 'none', // ensures it's hidden on iOS too
        //   },
        //   default: { display: 'none' },
        // }),
      }}>
      <Tabs.Screen
        name="explore"
        options={{
          title: 'Explore',
          // You can remove the icon option since the tab bar is hidden,
          // but it's okay to leave it if you might need it elsewhere.
          // tabBarIcon: ({ color }) => <IconSymbol size={28} name="paperplane.fill" color={color} />,
        }}
      />
    </Tabs>
  );
}
